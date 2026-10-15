"""Oriented 50 x 33 cm footprint, metres; geometry only, no hardware imports."""
import math

LARGO=.50
ANCHO=.33

def local(px,py,x,y,theta):
    c,s=math.cos(theta),math.sin(theta)
    return (px-x)*c+(py-y)*s,-(px-x)*s+(py-y)*c

def circulo(px,py,r,x,y,theta):
    a,b=local(px,py,x,y,theta)
    return max(abs(a)-LARGO/2,0)**2+max(abs(b)-ANCHO/2,0)**2 <= r*r

def segmento(ax,ay,bx,by,x,y,theta):
    a,b=local(ax,ay,x,y,theta); c,d=local(bx,by,x,y,theta)
    lo,hi=0.,1.
    for p,v,h in [(a,c-a,LARGO/2),(b,d-b,ANCHO/2)]:
        if abs(v)<1e-12:
            if abs(p)>h:return False
        else:
            q1,q2=(-h-p)/v,(h-p)/v
            lo=max(lo,min(q1,q2));hi=min(hi,max(q1,q2))
            if lo>hi:return False
    return True

def choca(mundo,x,y,theta):
    return any(segmento(*p,x,y,theta) for p in mundo.paredes) or any(
        circulo(o[0],o[1],o[2],x,y,theta) for o in mundo.objetos if o[4]>.5)

def barrido(mundo,x,y,theta,nx,ny,ntheta):
    # At most 5 mm travel of the most distant corner between checks.
    da=math.atan2(math.sin(ntheta-theta),math.cos(ntheta-theta))
    n=max(1,math.ceil((math.hypot(nx-x,ny-y)+math.hypot(LARGO,ANCHO)/2*abs(da))/.005))
    for i in range(1,n+1):
        f=i/n
        if choca(mundo,x+(nx-x)*f,y+(ny-y)*f,theta+da*f):return True
    return False
