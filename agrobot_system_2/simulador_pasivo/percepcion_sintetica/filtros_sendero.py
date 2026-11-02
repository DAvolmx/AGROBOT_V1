import numpy as np, cv2, json, sys
d=np.load('/home/jetson/Desktop/AGROBOT/agrobot_system_2/simulador_pasivo/percepcion_sintetica/datos/real/sendero.npz')
rgb,etq,ses=d['rgb'],d['etq'],d['sesion']
TEST=['20261028_200833_d17537','20261029_142951_a81509']
def box(x,k): return cv2.boxFilter(x,-1,(k,k))
def lstd(x,k): return np.sqrt(np.maximum(box(x*x,k)-box(x,k)**2,0))
def feats(im):
    b,g,r=[im[...,i].astype(np.float32) for i in (2,1,0)]  # im is RGB -> bgr order trick below
    R,G,B=[im[...,i].astype(np.float32) for i in range(3)]
    bgr=cv2.cvtColor(im,cv2.COLOR_RGB2BGR)
    lab=cv2.cvtColor(bgr,cv2.COLOR_BGR2LAB).astype(np.float32); hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV).astype(np.float32)
    gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY).astype(np.float32)
    s=R+G+B+1e-3; rn,gn,bn=R/s,G/s,B/s
    F={}
    F['L']=lab[...,0]; F['a']=lab[...,1]; F['b']=lab[...,2]
    F['H']=hsv[...,0]; F['S']=hsv[...,1]; F['V']=hsv[...,2]
    F['ExG']=2*gn-rn-bn; F['ExGR']=(2*gn-rn-bn)-(1.4*rn-gn); F['G/R']=G/(R+1)
    for k in (5,9,15): F[f'std_L{k}']=lstd(lab[...,0],k)
    F['std_a9']=lstd(lab[...,1],9)
    lap=np.abs(cv2.Laplacian(cv2.GaussianBlur(gray,(3,3),0),cv2.CV_32F)); F['lap9']=box(lap,9); F['lap21']=box(lap,21)
    gx=cv2.Sobel(gray,cv2.CV_32F,1,0,ksize=3); gy=cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3)
    mag=np.hypot(gx,gy); F['sobel9']=box(mag,9); F['sobel_v9']=box(np.abs(gx),9); F['sobel_h9']=box(np.abs(gy),9)
    gb=cv2.getGaborKernel((15,15),3,0,8,0.5); F['gabor0']=box(np.abs(cv2.filter2D(gray,cv2.CV_32F,gb)),9)
    gb=cv2.getGaborKernel((15,15),3,np.pi/2,8,0.5); F['gabor90']=box(np.abs(cv2.filter2D(gray,cv2.CV_32F,gb)),9)
    F['canny_dens']=box(cv2.Canny(gray.astype(np.uint8),40,100).astype(np.float32)/255,15)
    F['lab_dist_ref']=None  # ref dist (below)
    # distancia a la referencia (franja de abajo) normalizada: método de apariencia
    ref=np.dstack([lab[...,1],lab[...,2],lab[...,0],lstd(lab[...,0],9)])[int(240*.8):int(240*.98),int(320*.36):int(320*.64)].reshape(-1,4)
    med=np.median(ref,0); mad=np.maximum(np.median(np.abs(ref-med),0)*1.4826,[2,2,6,1.5])
    X=np.dstack([lab[...,1],lab[...,2],lab[...,0],lstd(lab[...,0],9)]); F['apariencia']=-np.sqrt((((X-med)/mad)**2*np.array([1,1,.35,1.2])**2).sum(2))
    F['color_ref']=-np.sqrt((((X[...,:3]-med[:3])/mad[:3])**2*np.array([1,1,.35])**2).sum(2))
    F['textura_ref']=-np.abs(X[...,3]-med[3])/mad[3]
    F['fila']=np.tile(np.arange(240,dtype=np.float32)[:,None],(1,320))
    return {k:v for k,v in F.items() if v is not None}
def auc(pos,neg):
    if len(pos)<50 or len(neg)<50: return np.nan
    x=np.concatenate([pos,neg]); r=x.argsort().argsort()+1
    return (r[:len(pos)].sum()-len(pos)*(len(pos)+1)/2)/(len(pos)*len(neg))
rng=np.random.default_rng(0); names=None
bandas=[(100,140),(140,180),(180,240)]
acum={}  # feat -> list per band of (pos arrays, neg arrays) test only
Xtr,ytr=[],[]; Xte,yte=[],[]
for i in range(len(rgb)):
    F=feats(rgb[i]); names=list(F)
    e=etq[i]; pos=(e==1); neg=(e==2)|(e==4)
    for k,(ys,xs) in enumerate([np.nonzero(pos),np.nonzero(neg)]):
        if len(ys)==0: continue
        j=rng.choice(len(ys),min(len(ys),150),replace=False); ys,xs=ys[j],xs[j]
        v=np.stack([F[n][ys,xs] for n in names],1)
        (Xte if ses[i] in TEST else Xtr).append(np.c_[v,ys,np.full(len(ys),1-k)])
Xtr=np.concatenate(Xtr); Xte=np.concatenate(Xte)
ni=len(names); res={}
for n,fi in zip(names,range(ni)):
    a_all=auc(Xte[Xte[:,-1]==1,fi],Xte[Xte[:,-1]==0,fi]); a_all=max(a_all,1-a_all) if n!='fila' else a_all
    ab=[]
    for lo,hi in bandas:
        m=(Xte[:,ni]>=lo)&(Xte[:,ni]<hi); p,q=Xte[m&(Xte[:,-1]==1),fi],Xte[m&(Xte[:,-1]==0),fi]
        ab.append(auc(p,q))
    res[n]={'auc_total':round(float(a_all),3),'auc_bandas':[None if np.isnan(x) else round(float(x),3) for x in ab]}
# signo: usar el de entrenamiento para orientar cada rasgo
for fi,n in enumerate(names):
    s=1 if auc(Xtr[Xtr[:,-1]==1,fi],Xtr[Xtr[:,-1]==0,fi])>=0.5 else -1
    res[n]['signo']=s
    ab=[]
    for lo,hi in bandas:
        m=(Xte[:,ni]>=lo)&(Xte[:,ni]<hi); p,q=Xte[m&(Xte[:,-1]==1),fi]*s,Xte[m&(Xte[:,-1]==0),fi]*s; ab.append(auc(p,q))
    ok=[x for x in ab if not np.isnan(x)]
    res[n]['auc_orientado_bandas']=[None if np.isnan(x) else round(float(x),3) for x in ab]
    res[n]['auc_medio_bandas']=round(float(np.mean(ok)),3) if ok else None
# combinación logística (ridge simple) sobre rasgos sin 'fila'
use=[i for i,n in enumerate(names) if n!='fila']
mu,sd=Xtr[:,use].mean(0),Xtr[:,use].std(0)+1e-6
A=(Xtr[:,use]-mu)/sd; A=np.c_[A,np.ones(len(A))]; y=Xtr[:,-1]
w=np.zeros(A.shape[1])
for _ in range(400):
    p=1/(1+np.exp(-A@w)); w-=0.5*(A.T@(p-y)/len(y)+1e-3*w)
B=(Xte[:,use]-mu)/sd; sc=np.c_[B,np.ones(len(B))]@w
comb=[]
for lo,hi in bandas:
    m=(Xte[:,ni]>=lo)&(Xte[:,ni]<hi); comb.append(round(float(auc(sc[m&(Xte[:,-1]==1)],sc[m&(Xte[:,-1]==0)])),3))
res['COMBINACION_logistica']={'auc_orientado_bandas':comb,'auc_medio_bandas':round(float(np.mean(comb)),3),'pesos':{names[u]:round(float(wi),2) for u,wi in zip(use,w[:-1])}}
json.dump({'names':names,'res':res,'mu':mu.tolist(),'sd':sd.tolist(),'w':w.tolist(),'use':[names[u] for u in use]},open('/home/jetson/Desktop/AGROBOT/agrobot_system_2/docs/SESION_CAMPO_20261029/filtros_sendero.json','w'),indent=1,ensure_ascii=False)
tab=sorted([(v['auc_medio_bandas'] if v.get('auc_medio_bandas') is not None else 0,k,v['auc_orientado_bandas']) for k,v in res.items()],reverse=True)
for m,k,b in tab: print(f'{k:22s} medio {m:.3f}  bandas {b}')
print('n test',len(Xte),'train',len(Xtr))
