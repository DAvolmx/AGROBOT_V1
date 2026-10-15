"""Reproducible controller parameter training in the rendered garden.
Training/validation/test seeds disjoint. Optimizes a route tracker, NOT vision.
SLAM here is the inherited noisy odometry + lidar occupancy mapper, closures
disabled to avoid ground-truth correction. This is not slam_toolbox.
"""
import argparse, csv, json, math, time
from pathlib import Path
from dataclasses import asdict
import numpy as np
from dinamica_pasiva import DinamicaPasiva, Parametros
import huella_pasiva as footprint
from mundo import Mundo
from slam_sim import SlamSimulado
from seguimiento_sendero import SeguidorSendero

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'resultados_pasivo'

def angle(x):return math.atan2(math.sin(x),math.cos(x))

def rollout(controller,seed,seconds=45,capture=False,drive='all'):
    rng=np.random.default_rng(seed)
    data=json.loads((ROOT/'mundos/huerta_fotos_50cm.json').read_text())
    world=Mundo.desde_dict(data)
    from ruta_ideal import trazar
    route=trazar(data['sendero'],paso=.05,cerrar=True)
    segments=np.diff(route,axis=0);lengths=np.linalg.norm(segments,axis=1)
    cumulative=np.r_[0,np.cumsum(lengths)]
    start=int(rng.integers(0,len(route)-1))
    heading=math.atan2(*segments[start][::-1])
    centre=route[start]+rng.normal(0,.015,2)
    follower=SeguidorSendero(route,closed=True,progress_m=float(cumulative[start]))
    p=Parametros(mu=float(rng.uniform(.5,.8)),pivot_damping=float(rng.uniform(.025,.065)),drive=drive)
    d=DinamicaPasiva((* (centre-.185*np.array([math.cos(heading),math.sin(heading)])),heading),p)
    slam=SlamSimulado((*centre,heading),world.limites,semilla=seed+2000,ruido=1.,cierres=False)
    slam.fallos_casamiento=False
    previous=centre.copy();previous_heading=heading
    rows=[];collisions=0;contact=False;travel=0.;plants=set();visited=set()
    dt=.05;linear=angular=0.;delay_queue=[]
    for step in range(round(seconds/dt)):
        # Route is a supplied map, not visual perception. Controller sees SLAM pose.
        estimated=np.array([slam.x,slam.y]);psi=slam.theta
        demand_v,demand_w=follower.orden((slam.x,slam.y,psi),speed=controller['speed'],
                                         lookahead=controller['lookahead'],gain=controller['gain'])
        # Fixed per-episode transport latency, same for compared controllers.
        delay=int(seed%3)+1
        delay_queue.append((demand_v,demand_w))
        if len(delay_queue)>delay:linear,angular=delay_queue.pop(0)
        old=d.q.copy();d.step(dt,linear,angular)
        theta=d.q[2];centre=d.q[:2]+.185*np.array([math.cos(theta),math.sin(theta)])
        hit=footprint.barrido(world,*previous,previous_heading,*centre,theta,old[3],d.q[3])
        if hit:
            if not contact:collisions+=1
            d.q[:]=old;d.dq[:]=0;d.wheels[:]=0
            centre=previous.copy();theta=previous_heading
        contact=hit
        displacement=centre-previous;travel+=float(np.linalg.norm(displacement))
        forward=float(displacement@np.array([math.cos(theta),math.sin(theta)])/dt)
        yaw=angle(theta-previous_heading)/dt
        slam.paso(dt,forward,yaw,(*centre,theta))
        # scans during captured evaluation only: same estimator in all runs.
        if capture and step%4==0:
            points=world.scan(*centre,theta,n=180,rng=rng,realismo=1.,omega=yaw)
            slam.ingerir_scan(points,omega=yaw)
        ab=segments;proj=np.clip(((centre-route[:-1])*ab).sum(axis=1)/np.maximum(lengths**2,1e-9),0,1)
        dist=np.linalg.norm(centre-(route[:-1]+proj[:,None]*ab),axis=1)
        idx=int(dist.argmin());cross=float(dist[idx]);visited.add(idx)
        for i,plant in enumerate(world.plantas):
            if footprint.circulo(plant[0],plant[1],0,*centre,theta,d.q[3]):plants.add(i)
        rows.append([step*dt,*centre,theta,float(d.q[3]),cross,slam.x,slam.y,slam.error((*centre,theta)),d.slip,linear,angular])
        previous,previous_heading=centre.copy(),theta
    a=np.asarray(rows)
    # Conservative whole-robot clearance test against curved route sampling.
    metrics={'seed':seed,'seconds':seconds,'cross_track_rmse_m':float(np.sqrt(np.mean(a[:,5]**2))),
             'cross_track_p95_m':float(np.percentile(a[:,5],95)),
             'centre_in_path_pct':float(np.mean(a[:,5]<=.25)*100),
             'nominal_clearance_pct':float(np.mean(a[:,5]<=.065)*100),
             'collisions':collisions,'plant_contacts':len(plants),'distance_m':travel,
             'route_coverage_pct':float(sum(lengths[list(visited)])/sum(lengths)*100),
             'slam_ate_rmse_m':float(np.sqrt(np.mean(a[:,8]**2))),
             'max_delta_deg':float(np.degrees(np.max(np.abs(a[:,4])))),
             'mean_slip_mps':float(a[:,9].mean()),'parameters':asdict(p)}
    metrics['cost']=metrics['cross_track_rmse_m']+2*collisions+.2*len(plants)+.5*(1-metrics['route_coverage_pct']/100)
    if capture:
        import cv2
        OUT.mkdir(exist_ok=True)
        stem=f"{controller['name']}_seed{seed}"
        with (OUT/f'{stem}.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['time_s','x','y','yaw_rad','delta_rad','cross_track_m','slam_x','slam_y','slam_error_m','slip_mps','command_v','command_w']);writer.writerows(rows)
        cv2.imwrite(str(OUT/f'{stem}_slam.png'),cv2.resize(slam.mapa_imagen(),None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
        np.save(OUT/f'{stem}_occupancy.npy',slam.rejilla)
    return metrics

def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=45);parser.add_argument('--drive',choices=('all','rear','front'),default='all');parser.add_argument('--salida',default=str(OUT));args=parser.parse_args()
    OUT=Path(args.salida).expanduser().resolve()
    OUT.mkdir(parents=True,exist_ok=True);started=time.time()
    baseline={'name':'baseline','speed':.20,'lookahead':.35,'gain':1.2}
    candidates=[baseline]+[{'name':f'candidate{i}','speed':.16,'lookahead':look,'gain':gain}
                          for i,(look,gain) in enumerate([( .25,2.),(.4,2.),(.6,2.),(.4,3.),(.6,3.)])]
    history=[]
    for c in candidates:
        scores=[rollout(c,s,args.seconds,drive=args.drive) for s in (11,12)]
        result={'controller':c,'mean_cost':float(np.mean([m['cost'] for m in scores])),'episodes':scores}
        history.append(result);print(json.dumps({'training':c['name'],'cost':result['mean_cost']}),flush=True)
        (OUT/'training_progress.json').write_text(json.dumps(history,indent=2))
    baseline_record=history[0]
    best_candidate=min(history[1:],key=lambda r:r['mean_cost'])
    finalists=[baseline_record] if best_candidate['mean_cost']>=baseline_record['mean_cost'] else [baseline_record,best_candidate]
    validation=[]
    for f in finalists:
        c=f['controller'];ms=[rollout(c,s,args.seconds,drive=args.drive) for s in (101,102)]
        validation.append({'controller':c,'mean_cost':float(np.mean([m['cost'] for m in ms])),'episodes':ms})
    best_validation=min(validation,key=lambda r:r['mean_cost'])
    baseline_validation=next(r for r in validation if r['controller']['name']=='baseline')
    no_improvement=best_validation['mean_cost']>=baseline_validation['mean_cost']-1e-9
    winner=dict(baseline if no_improvement else best_validation['controller'])
    winner['name']='baseline_retained' if no_improvement else 'trained'
    test=[]
    for c in (baseline,winner):
        for seed in (1001,1002,1003):
            m=rollout(c,seed,args.seconds,capture=True,drive=args.drive);m['controller']=c['name'];test.append(m)
            print(json.dumps({'test':c['name'],'seed':seed,'rmse':m['cross_track_rmse_m'],'collisions':m['collisions']}),flush=True)
    report={'method':'local-progress pure-pursuit grid search; no neural or pest training',
            'training_outcome':'baseline_retained' if no_improvement else 'candidate_improved',
            'baseline_validation_cost':float(baseline_validation['mean_cost']),
            'winner_validation_cost':float(best_validation['mean_cost']),
            'validation_improvement_fraction':float(max(0.,(baseline_validation['mean_cost']-best_validation['mean_cost'])/max(abs(baseline_validation['mean_cost']),1e-9))),
            'scene':'huerta_fotos_50cm','train_seeds':[11,12],'validation_seeds':[101,102],
            'test_seeds':[1001,1002,1003],'history':history,'validation':validation,'test':test,
            'candidate':winner,'wall_seconds':time.time()-started,'drive_assumption':args.drive,
            'slam':'noisy odometry plus lidar occupancy, no scan matching or loop closure',
            'real_deployment':False}
    (OUT/'metrics.json').write_text(json.dumps(report,indent=2))
    (OUT/'controlador_candidato.json').write_text(json.dumps(winner,indent=2))
    print('FINISHED',report['wall_seconds'],flush=True)
if __name__=='__main__':main()
