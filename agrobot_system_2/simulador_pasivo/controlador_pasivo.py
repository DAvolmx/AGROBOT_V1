"""Route tracker trained by mini_entrenamiento, continuous wheel commands.
Consumes an estimated pose and a supplied route. No hardware imports.
"""
import math
import numpy as np

def ordenar(route,pose,config):
    route=np.asarray(route,float)
    x,y,psi=pose
    if len(route)<2:return 0.,0.
    nearest=int(np.argmin(np.linalg.norm(route[:-1]-[x,y],axis=1)))
    lengths=np.linalg.norm(np.diff(route,axis=0),axis=1)
    index=nearest;travel=0.
    for _ in range(len(lengths)):
        if travel>=config['lookahead']:break
        travel+=lengths[index];index=(index+1)%len(lengths)
    e=math.atan2(route[index,1]-y,route[index,0]-x)-psi
    e=math.atan2(math.sin(e),math.cos(e))
    return config['speed']/(1+1.8*abs(e)),float(np.clip(config['gain']*e,-1.2,1.2))
