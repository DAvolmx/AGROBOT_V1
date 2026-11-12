import cv2,numpy as np
def detectar_moscas(img, lado=1600, th_min=30, L_min=165, s_max=60):
    """Filtro clásico de mosca blanca: top-hat blanco sobre la luminancia (puntos claros del tamaño de un insecto), solo dentro de la hoja
    (erosionada para quitar bordes), descartando trazos alargados (nervaduras) y agrupando los puntos en colonias. Devuelve dict + imagen."""
    h,w=img.shape[:2]; k=lado/max(h,w); im=cv2.resize(img,(int(w*k),int(h*k)),interpolation=cv2.INTER_AREA) if k<1 else img.copy()
    L=cv2.cvtColor(im,cv2.COLOR_BGR2LAB)[...,0]; hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    hoja=cv2.inRange(hsv,(30,25,60),(100,255,255)); hoja=cv2.morphologyEx(hoja,cv2.MORPH_CLOSE,np.ones((21,21),np.uint8))
    hoja=cv2.erode(hoja,np.ones((25,25),np.uint8))
    r=max(3,int(round(9*im.shape[1]/1600)))
    th=cv2.morphologyEx(L,cv2.MORPH_TOPHAT,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*r+1,2*r+1)))
    m=((th>th_min)&(L>L_min)&(hsv[...,1]<s_max)&(hoja>0)).astype(np.uint8)*255
    n,_,st,cen=cv2.connectedComponentsWithStats(m)
    pts=[]
    for i in range(1,n):
        x,y,bw,bh,a=st[i]
        if not (2<=a<=(3*r)**2): continue
        if max(bw,bh)/max(1,min(bw,bh))>2.5 or a/(bw*bh)<0.35: continue      # trazo alargado o hueco: nervadura, borde
        pts.append(cen[i])
    pts=np.array(pts) if pts else np.zeros((0,2))
    R=6*r; colonias=[]
    usados=np.zeros(len(pts),bool)
    for i in range(len(pts)):
        if usados[i]: continue
        d=np.hypot(*(pts-pts[i]).T); grupo=np.where((d<R)&~usados)[0]
        if len(grupo)>=5: usados[grupo]=True; colonias.append(pts[grupo])
    area_hoja=max(1,int((hoja>0).sum()))
    vis=im.copy()
    for p in pts: cv2.circle(vis,(int(p[0]),int(p[1])),r,(0,255,255),1)
    for c in colonias:
        x1,y1=c.min(0)-r; x2,y2=c.max(0)+r; cv2.rectangle(vis,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),3)
    return {"puntos":len(pts),"colonias":len(colonias),"moscas_en_colonias":int(sum(len(c) for c in colonias)),
            "densidad":round(len(pts)/area_hoja*1e5,1)}, vis
