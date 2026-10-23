"""Field capture: bounded asynchronous queue; no actuator commands."""
import asyncio, json, os, time, uuid, threading, queue, shutil, zipfile
from pathlib import Path
from aiohttp import web
import cv2
import math

def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):
        return [finite_json(v) for v in value]
    return value


class Recoleccion:
    def __init__(self, snapshot, frame):
        self.snapshot, self.frame = snapshot, frame
        self.root = Path.home()/'.agrobot'/'campo'
        self.root.mkdir(parents=True, exist_ok=True)
        self.active=False; self.video=False; self.directory=None
        self.q=queue.Queue(maxsize=20); self.lock=threading.Lock()
        self.count=0; self.dropped=0; self.error=None; self.writer=None
        self.thread=threading.Thread(target=self.work,daemon=True);self.thread.start()

    def status(self):
        try:
            live=self.snapshot()
            sensors={k:live.get(k) for k in ('camera','lidar_age_s','pose_age_s','pose_reliable')}
        except Exception:sensors={}
        return finite_json(dict(active=self.active,video=self.video,session=self.directory.name if self.directory else None,
                    samples=self.count,dropped=self.dropped,error=self.error,queue=self.q.qsize(),
                    free_gb=round(shutil.disk_usage(self.root).free/1e9,1),rate_hz=5,video_fps=10,sensors=sensors))

    def submit(self, kind, data=None):
        try:self.q.put_nowait((kind,data))
        except queue.Full:self.dropped+=1;raise RuntimeError('Cola llena; espere antes de repetir')

    async def api(self, request):
        if request.method=='GET':return web.json_response(self.status())
        action=request.match_info['action']
        try:
            if action=='start':
                if not self.active:
                    if self.q.unfinished_tasks:raise RuntimeError('Escritura anterior pendiente')
                    if shutil.disk_usage(self.root).free<5e9:raise RuntimeError('Menos de 5 GB libres')
                    self.directory=self.root/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
                    self.directory.mkdir();(self.directory/'fotos').mkdir()
                    self.count=0;self.dropped=0;self.error=None
                    (self.directory/'manifest.json').write_text(json.dumps(dict(schema='agrobot.field.v1',
                        started_unix=time.time(),context='field_unlabelled',labels=False,
                        synchronization='nearest available sensor sample; ages recorded, not hardware synchronized',
                        config=finite_json(self.snapshot())),default=str,indent=2,allow_nan=False))
                    self.active=True
            elif action=='stop':
                self.active=False;self.video=False;self.submit('close',self.directory)
            elif action=='photo':
                if not self.active:raise RuntimeError('Inicie recolección primero')
                f=self.frame()
                if f is None:raise RuntimeError('Cámara sin imagen')
                self.submit('photo',(self.directory,f,time.time()))
            elif action=='video':
                if not self.active:raise RuntimeError('Inicie recolección primero')
                if self.frame() is None:raise RuntimeError('Cámara sin imagen')
                self.video=not self.video
                if not self.video:self.submit('video_close')
            else:raise RuntimeError('Acción desconocida')
        except (RuntimeError,OSError) as e:return web.json_response({'error':str(e)},status=409)
        return web.json_response(self.status())

    async def loop(self):
        last_photo=0;last_seq=None
        while True:
            if self.active:
                if shutil.disk_usage(self.root).free<5e9:
                    self.active=False;self.video=False;self.error='Disco bajo';self.submit('close',self.directory)
                else:
                    f=self.frame();now=time.monotonic()
                    data=self.snapshot();data.pop('configuration',None);data.update(t_unix=time.time(),t_monotonic=now,
                        camera_timestamp_monotonic=f[1] if f else None,camera_age_s=now-f[1] if f else None,
                        camera_sequence=f[2] if f else None)
                    try:
                        self.submit('sample',(self.directory,data))
                        if f and f[2]!=last_seq:
                            last_seq=f[2]
                            if now-last_photo>=.5:
                                self.submit('photo',(self.directory,f,time.time()));last_photo=now
                    except RuntimeError:pass
            await asyncio.sleep(.2)

    async def video_loop(self):
        seq=None
        while True:
            if self.active and self.video:
                f=self.frame()
                if f and f[2]!=seq:
                    seq=f[2]
                    try:self.submit('video',(self.directory,f,time.time()))
                    except RuntimeError:pass
            await asyncio.sleep(.1)

    def work(self):
        while True:
            kind,data=self.q.get()
            try:
                if kind=='sample':
                    d,s=data
                    with (d/'variables.jsonl').open('a') as out:
                        out.write(json.dumps(finite_json(s),default=str,allow_nan=False)+'\n')
                        # 2026-10-23: la Jetson se apagó por batería y la sesión quedó en 0 bytes (todo seguía en caché). fsync cada 2 s.
                        if time.monotonic()-getattr(self,'_t_fsync',0.0)>2.0:
                            out.flush();os.fsync(out.fileno());self._t_fsync=time.monotonic()
                    self.count+=1
                elif kind in ('photo','video'):
                    d,f,t=data;image,ts,seq=f
                    if kind=='photo':
                        name=f'fotos/{seq}_{t:.6f}.jpg'
                        if not cv2.imwrite(str(d/name),image,[cv2.IMWRITE_JPEG_QUALITY,85]):raise OSError('JPEG falló')
                    else:
                        if self.writer is None:
                            name=f'video_{time.time_ns()}.avi';self.video_name=name
                            h,w=image.shape[:2];self.writer=cv2.VideoWriter(str(d/name),cv2.VideoWriter_fourcc(*'MJPG'),10,(w,h))
                            if not self.writer.isOpened():self.writer.release();self.writer=None;raise OSError('Codec MJPG no disponible')
                        self.writer.write(image);name=self.video_name
                    with (d/'frames.jsonl').open('a') as out:out.write(json.dumps(dict(kind=kind,file=name,sequence=seq,capture_monotonic=ts,observed_unix=t))+'\n')
                elif kind in ('close','video_close'):
                    if self.writer:self.writer.release();self.writer=None
                    if kind=='close' and data:
                        (data/'summary.json').write_text(json.dumps(self.status(),indent=2))
                        with zipfile.ZipFile(data.with_suffix('.zip'),'w',zipfile.ZIP_STORED) as z:
                            for p in data.rglob('*'):
                                if p.is_file():z.write(p,p.relative_to(data))
            except Exception as e:self.error=str(e);self.video=False
            finally:self.q.task_done()

    async def download(self,request):
        if self.active or self.q.unfinished_tasks:return web.json_response({'error':'Detenga la sesión y espere el empaquetado'},status=409)
        if not self.directory or not self.directory.with_suffix('.zip').exists():raise web.HTTPNotFound()
        return web.FileResponse(self.directory.with_suffix('.zip'),headers={'Content-Disposition':f'attachment; filename="{self.directory.name}.zip"'})

def install(app,snapshot,frame):
    r=Recoleccion(snapshot,frame)
    app.router.add_get('/api/recoleccion',r.api)
    app.router.add_post('/api/recoleccion/{action}',r.api)
    app.router.add_get('/api/recoleccion-export',r.download)
    async def start(app):app['capture_tasks']=[asyncio.create_task(r.loop()),asyncio.create_task(r.video_loop())]
    async def stop(app):
        r.active=False;r.video=False
        for t in app.get('capture_tasks',[]):t.cancel()
        r.submit('close',r.directory)
        await asyncio.to_thread(r.q.join)
    app.on_startup.append(start);app.on_cleanup.append(stop)
    return r
