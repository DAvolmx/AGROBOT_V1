#!/usr/bin/env python3
"""Optimize PP parameters in Gazebo, with paired seeds and held-out validation.

This trains controller parameters, not neural-network weights. Ground truth is
reserved for scoring; camera and LiDAR alone feed the PP controller.
"""
import argparse
import datetime as dt
import fcntl
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
MODES = ('surco', 'barro', 'baches', 'pendiente', 'noche', 'bajo', 'mixto')
BASELINE = {'controller': 'pp', 'speed': .04, 'lookahead': .70}


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def atomic(path, obj):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(obj, indent=2, ensure_ascii=False))
    temp.replace(path)


def valid(m):
    return (m.get('evaluation_pose_source') == 'gazebo_ground_truth'
            and len(m.get('ground_truth_trajectory', [])) >= 10
            and m.get('camera_frames', 0) >= 5
            and m.get('cross_track_rmse_cm') is not None)


def safe_better(candidate, baseline):
    """All paired cases must avoid safety and progress regressions."""
    if not candidate or len(candidate) != len(baseline):
        return False
    for c, b in zip(candidate, baseline):
        if not valid(c) or not valid(b):
            return False
        if c['obstacle_contact_events'] > b['obstacle_contact_events']:
            return False
        if c['cross_track_p95_cm'] > b['cross_track_p95_cm'] + .5:
            return False
        if c['cross_track_rmse_cm'] > b['cross_track_rmse_cm'] + .5:
            return False
        if c['within_6cm_pct'] < b['within_6cm_pct'] - 1:
            return False
        if c['distance_x_m'] < max(.1, b['distance_x_m'] * .95):
            return False
    def score(rows):
        return sum(m['progress_m_per_min'] - .12*m['cross_track_rmse_cm']
                   - 3*m['obstacle_contact_events'] for m in rows)/len(rows)
    return score(candidate) > score(baseline) + .025


class Trainer:
    def __init__(self, args):
        self.args = args
        self.root = args.run_dir.resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.children = []
        self.logs = []
        self.started = time.monotonic()
        self.deadline = self.started + args.hours * 3600
        self.best = dict(BASELINE)
        self.episodes = 0
        self.completed = 0
        self.accepted = 0
        self.state = {'status': 'starting', 'started_utc': now(),
                      'planned_end_utc': (dt.datetime.now(dt.timezone.utc)
                                          +dt.timedelta(hours=args.hours)).isoformat(),
                      'duration_hours': args.hours, 'controller': 'pp',
                      'learning_type': 'paired_parameter_optimization_not_neural_training',
                      'real_robot_deployment': False, 'ros_domain_id': 42}
        if (self.root/'best_parameters.json').exists():
            self.best = json.loads((self.root/'best_parameters.json').read_text())['parameters']
        if (self.root/'status.json').exists():
            old = json.loads((self.root/'status.json').read_text())
            self.state['started_utc'] = old['started_utc']
            self.state['planned_end_utc'] = old['planned_end_utc']
            remaining = (dt.datetime.fromisoformat(old['planned_end_utc'])
                         - dt.datetime.now(dt.timezone.utc)).total_seconds()
            self.deadline = time.monotonic() + remaining
            elapsed = (dt.datetime.now(dt.timezone.utc)
                       - dt.datetime.fromisoformat(old['started_utc'])).total_seconds()
            self.started = time.monotonic() - elapsed
            self.episodes = old.get('episodes_started', old.get('completed_episodes', 0))
            self.accepted = old.get('accepted_candidates', 0)
            history = self.root/'episodes.jsonl'
            self.completed = sum(1 for _ in history.open()) if history.exists() else 0
        self.save()

    def save(self, **updates):
        self.state.update(updates)
        self.state.update(updated_utc=now(), completed_episodes=self.completed,
                          episodes_started=self.episodes,
                          accepted_candidates=self.accepted, best_parameters=self.best,
                          elapsed_s=round(time.monotonic()-self.started, 1))
        atomic(self.root/'status.json', self.state)

    def stop_children(self):
        for p in reversed(self.children):
            if p.poll() is None:
                try: os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError: pass
        for p in reversed(self.children):
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                p.wait(timeout=5)
        self.children.clear()
        for f in self.logs: f.close()
        self.logs.clear()

    def spawn(self, cmd, log):
        f = log.open('w'); self.logs.append(f)
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True)
        self.children.append(p)
        return p

    def episode(self, params, mode, seed, label, duration=None):
        for retry in range(3):
            try:
                return self._episode(params, mode, seed,
                                     label if retry == 0 else f'{label}_retry{retry}', duration)
            except (TimeoutError, KeyboardInterrupt):
                raise
            except Exception as exc:
                self.stop_children()
                self.save(last_episode_error=repr(exc), episode_retry=retry)
                print(f'Episode retry {retry}: {exc!r}', flush=True)
                if retry == 2: raise
                time.sleep(3)

    def _episode(self, params, mode, seed, label, duration=None):
        if time.monotonic() + 30 >= self.deadline:
            raise TimeoutError('Training wall-clock budget reached')
        self.stop_children()
        self.episodes += 1
        folder = self.root/f'episode_{self.episodes:04d}_{label}_{mode}_{seed}'
        folder.mkdir()
        self.save(status='running', active_episode=str(folder.name), active_parameters=params,
                  active_scenario=mode, active_seed=seed)
        world = folder/'world.sdf'
        subprocess.run(['python3', str(HERE/'generar_escenario.py'), '--modo', mode,
                        '--semilla', str(seed), '--salida', str(world)], check=True,
                       timeout=30, stdout=subprocess.DEVNULL)
        tree = ET.parse(world)
        robot = tree.getroot().find(".//model[@name='agrobot']")
        rng = random.Random(seed)
        # Start nearer the obstacles; paired seeds share the same perturbation.
        robot.find('pose').text = f"-1.0 {rng.uniform(-.025,.025)} .31 0 0 {rng.uniform(-.05,.05)}"
        plugin = ET.SubElement(robot, 'plugin', filename='gz-sim-odometry-publisher-system',
                               name='gz::sim::systems::OdometryPublisher')
        for k, v in {'odom_frame':'world', 'robot_base_frame':'agrobot/base_link',
                     'odom_topic':'/agrobot/ground_truth/odom', 'dimensions':'3',
                     'odom_publish_frequency':'20', 'gaussian_noise':'0'}.items():
            ET.SubElement(plugin, k).text = v
        tree.write(world, encoding='utf-8', xml_declaration=True)
        bridge = folder/'bridge.yaml'
        bridge.write_text((HERE/'bridge.yaml').read_text()+'''\n- ros_topic_name: /agrobot/ground_truth/odom
  gz_topic_name: /agrobot/ground_truth/odom
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
''')
        gz = self.spawn(['gz','sim','--headless-rendering','-s','-r',str(world)], folder/'gazebo.log')
        time.sleep(4)
        if gz.poll() is not None: raise RuntimeError('Gazebo failed to start')
        bridge_proc = self.spawn(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args',
                                 '-p',f'config_file:={bridge}'],folder/'bridge.log')
        time.sleep(4)
        seconds = min(duration or self.args.episode_seconds, self.deadline-time.monotonic()-5)
        if seconds < 15: raise TimeoutError('Budget reached before episode')
        output = folder/'metrics.json'
        pilot = self.spawn(['python3',str(HERE/'seguir_sendero.py'),'--controller','pp',
                            '--duration',str(seconds),'--speed',str(params['speed']),
                            '--lookahead',str(params['lookahead']),'--salida',str(output),
                            '--capturas',str(folder/'capturas'),'--require-ground-truth'],
                            folder/'pilot.log')
        try:
            pilot.wait(timeout=seconds+30)
            if pilot.returncode != 0 or not output.exists():
                raise RuntimeError(f'Pilot failed: {folder}')
            if bridge_proc.poll() is not None or gz.poll() is not None:
                raise RuntimeError('Simulator or bridge exited during episode')
            metrics = json.loads(output.read_text())
            if not valid(metrics):
                raise RuntimeError(f'Insufficient ground truth / camera: {folder}')
            record = {'episode':self.episodes,'label':label,'mode':mode,'seed':seed,
                      'parameters':params,'utc':now(),
                      **{k:v for k,v in metrics.items() if k not in
                         ('trajectory','ground_truth_trajectory')}}
            self.completed += 1
            with (self.root/'episodes.jsonl').open('a') as f:
                f.write(json.dumps(record)+'\n')
            print(json.dumps(record), flush=True)
            self.save(last_metrics=record)
            return metrics
        finally:
            self.stop_children()

    def run(self):
        if not (self.root/'best_parameters.json').exists():
            atomic(self.root/'best_parameters.json', {'parameters':self.best,
                   'origin':'best_previous_short_trial_PP', 'verified_in_new_suite':False})
        if self.args.smoke:
            result = self.episode(self.best,'surco',20261013,'smoke',duration=20)
            self.save(status='smoke_passed', ground_truth_samples=len(result['ground_truth_trajectory']))
            return
        # First collect the reference on every terrain; no neural truth-label feed into control.
        for i, mode in enumerate(MODES):
            self.episode(self.best,mode,202610130+i,'initial_reference')
        rng = random.Random(20261013)
        attempt = 0
        while time.monotonic()+600 < self.deadline:
            attempt += 1
            candidate = {'controller':'pp', 'speed':rng.choice([.035,.04,.045,.05,.055,.06]),
                         'lookahead':rng.choice([.55,.70,.85,1.0,1.15])}
            if candidate == self.best: continue
            modes = [MODES[(attempt-1)%7], MODES[(attempt+2)%7]]
            base, proposed = [], []
            for i, mode in enumerate(modes):
                seed = 202700000+attempt*10+i  # training seeds
                base.append(self.episode(self.best,mode,seed,'train_baseline'))
                proposed.append(self.episode(candidate,mode,seed,'train_candidate'))
            accepted = False
            if safe_better(proposed,base) and time.monotonic()+650 < self.deadline:
                vb, vc = [], []
                for i, mode in enumerate(('mixto','noche','barro')):
                    seed = 302700000+attempt*10+i  # held-out seeds
                    vb.append(self.episode(self.best,mode,seed,'validation_baseline'))
                    vc.append(self.episode(candidate,mode,seed,'validation_candidate'))
                accepted = safe_better(vc,vb)
            if accepted:
                self.best = candidate; self.accepted += 1
                atomic(self.root/'best_parameters.json', {'parameters':self.best,'accepted_utc':now(),
                        'validated_attempt':attempt,'validation':'paired_unseen_seeds',
                        'real_robot_deployment':False})
            with (self.root/'candidates.jsonl').open('a') as f:
                f.write(json.dumps({'attempt':attempt,'candidate':candidate,'accepted':accepted,
                                    'utc':now()})+'\n')
            self.save(last_candidate_accepted=accepted)
        # Fill the remaining budget with stress evaluation of the current best.
        while time.monotonic()+45 < self.deadline:
            mode = MODES[self.episodes%7]
            self.episode(self.best,mode,402700000+self.episodes,'final_stress')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hours',type=float,default=4)
    p.add_argument('--episode-seconds',type=float,default=90)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--smoke',action='store_true')
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise SystemExit('ROS_DOMAIN_ID must be 42 for isolated simulation')
    lock = open(Path.home()/'.agrobot/gazebo_training.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    subprocess.run(['bash',str(HERE/'detener_ros_gazebo.sh')],check=True,timeout=30)
    trainer = Trainer(args)
    def stop(_signum,_frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:
        trainer.run()
        if not args.smoke: trainer.save(status='completed')
    except TimeoutError:
        trainer.save(status='completed',reason='four_hour_budget_reached')
    except KeyboardInterrupt:
        trainer.save(status='stopped',reason='signal')
    except Exception as exc:
        trainer.save(status='failed',error=repr(exc)); raise
    finally:
        trainer.stop_children()


if __name__ == '__main__': main()
