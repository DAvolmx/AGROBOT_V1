"""Allowlisted same-origin HTTP bridge to the C3 setup API. No arbitrary destinations."""
import asyncio
import os
import time
import aiohttp
from aiohttp import web

ACTIONS={'claim','stop','motor','map','verify','jetson','invert','actuator','drive','config','config_reset'}
FIELDS={'claim':set(),'stop':set(),'verify':set(),'jetson':set(),
 'motor':{'token','seq','m','d','p'},'map':{'r0','r1','r2','r3'},'invert':{'m'},
 'actuator':{'token','seq','servo','led'},'drive':{'token','seq','v','w'},
 'config':{'fwd','rev','minp','inner','slew','gap'},'config_reset':set()}
EXPECTED='demo-trasero-1.1'          # minimo aceptado (conserva compatibilidad)
ACCEPTED={'demo-trasero-1.1','demo-trasero-1.2'}
CONFIG_FIRMWARE='demo-trasero-1.2'   # la configuracion de limites exige >= esta
# Ruta en la C3 de cada accion (config_reset no se puede llamar igual).
PATHS={'config_reset':'/config/reset'}
# Mismos limites DUROS que el firmware. Se validan aqui tambien: el firmware es
# la ultima defensa, pero no se le manda nada que se sabe invalido.
CONFIG_BOUNDS={'fwd':(10,100),'rev':(10,100),'minp':(0,60),'inner':(0,90),'slew':(20,300),'gap':(10,200)}

class C3Setup:
    def __init__(self):
        self.url='http://192.168.12.240'
        self.auth=aiohttp.BasicAuth(os.getenv('AGROBOT_C3_USER','agrobot'),os.getenv('AGROBOT_C3_PASS','12345678'))
        self.checked=0.
        self.lock=asyncio.Lock()
        self.on_claim=None
        self.firmware=''
    async def call(self,method,path,data=None):
        async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=1.5)) as session:
            async with session.request(method,self.url+path,data=data,auth=self.auth,allow_redirects=False) as r:
                text=await r.text()
                if r.status!=200:raise ValueError(f'C3 HTTP {r.status}: {text[:160]}')
                if path=='/status':
                    import json
                    return json.loads(text)
                return text
    async def status(self):
        d=await self.call('GET','/status')
        self.firmware=d.get('firmware','')
        d['compatible']=self.firmware in ACCEPTED
        d['config_disponible']=self.firmware==CONFIG_FIRMWARE
        if d['compatible']:self.checked=time.monotonic()
        else:self.checked=0
        return d
    async def stop(self):
        try:
            async with self.lock:await self.call('POST','/halt',{})
            return True
        except Exception:return False
    async def get_handler(self,request):
        try:return web.json_response(await self.status())
        except Exception as e:return web.json_response({'error':str(e) or type(e).__name__,'connected':False,'compatible':False},status=503)
    async def post_handler(self,request):
        action=request.match_info['action']
        if action not in ACTIONS:return web.json_response({'error':'acción desconocida'},status=404)
        try:
            data=await request.json()
            if not isinstance(data,dict) or set(data)-FIELDS[action]:raise ValueError('parámetros inválidos')
            if any(not isinstance(v,(str,int)) or isinstance(v,bool) for v in data.values()):raise ValueError('valores inválidos')
            async with self.lock:
                if action!='stop' and time.monotonic()-self.checked>2:
                    if not (await self.status())['compatible']:
                        return web.json_response({'error':'Carga el firmware demo-trasero-1.2 (o 1.1) antes de configurar'},status=409)
                if action in {'config','config_reset'}:
                    if self.firmware!=CONFIG_FIRMWARE:
                        return web.json_response({'error':'La configuracion de limites exige el firmware '+CONFIG_FIRMWARE},status=409)
                    if action=='config':
                        if not data:raise ValueError('sin parametros')
                        for k,v in data.items():
                            lo,hi=CONFIG_BOUNDS[k]
                            try:iv=int(v)
                            except (TypeError,ValueError):raise ValueError(f'{k}: no es un entero')
                            if not lo<=iv<=hi:raise ValueError(f'{k} fuera de rango ({lo}..{hi})')
                            data[k]=iv
                        if data.get('minp',0)>data.get('fwd',100):raise ValueError('minp no puede superar fwd')
                if action in {'claim','map','invert','verify','jetson','config','config_reset'} and self.on_claim:
                    self.on_claim()
                text=await self.call('POST',PATHS.get(action,'/'+action),data)
            return web.json_response({'ok':True,'value':text})
        except (ValueError,TypeError) as e:return web.json_response({'error':str(e)},status=400)
        except Exception as e:return web.json_response({'error':'C3 no disponible: '+(str(e) or type(e).__name__)},status=503)
