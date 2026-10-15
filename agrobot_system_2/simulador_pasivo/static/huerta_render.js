// Photo-inspired assets. Dimensions of the footprint are measured; heights are approximate.
function glRobot(color, fantasma) {
  const g=new THREE.Group();
  const mat=(c,metal=0)=>new THREE.MeshStandardMaterial({color:c,roughness:.65,metalness:metal,transparent:!!fantasma,opacity:fantasma?.28:1});
  const red=mat(0x9e251c,.35), black=mat(0x202326,.3), rubber=mat(0x17191a), steel=mat(0x878b85,.7);
  function mesh(geo,m,x,y,z){const o=new THREE.Mesh(geo,m);o.position.set(x,y,z);o.castShadow=!fantasma;o.receiveShadow=true;g.add(o);return o;}
  function box(a,b,c,m,x,y,z){return mesh(new THREE.BoxGeometry(a,b,c),m,x,y,z);}
  box(.35,.035,.23,red,0,.195,0);
  box(.33,.10,.235,black,-.015,.27,0);
  box(.35,.013,.25,steel,-.015,.327,0);
  mesh(new THREE.CylinderGeometry(.038,.038,.032,24),black,-.04,.35,0);
  box(.07,.013,.018,black,.11,.34,0);
  for(const x of [-.153,.153])for(const z of [-.1375,.1375]){
    const w=mesh(new THREE.CylinderGeometry(.086,.086,.047,24),rubber,x,.092,z);w.rotation.x=Math.PI/2;
    const hub=mesh(new THREE.CylinderGeometry(.038,.038,.051,16),steel,x,.092,z);hub.rotation.x=Math.PI/2;
    for(let k=0;k<16;k++){
      const a=k*Math.PI/8;
      const lug=box(.025,.014,.055,rubber,x+Math.sin(a)*.091,.092+Math.cos(a)*.091,z);lug.rotation.z=-a;
    }
    box(.021,.11,.023,red,x,.185,z*.70);
    for(let k=0;k<8;k++){
      const spring=mesh(new THREE.TorusGeometry(.017,.003,5,14),red,x,.16+k*.008,z*.70);spring.rotation.x=Math.PI/2;
    }
  }
  box(.045,.075,.11,black,.188,.256,0);
  const lens=mesh(new THREE.CylinderGeometry(.019,.019,.014,20),mat(0x09171b,.8),.217,.245,0);lens.rotation.z=Math.PI/2;
  box(.008,.009,.082,new THREE.MeshStandardMaterial({color:0xfff3c5,emissive:0xaaa176}),.214,.279,0);
  for(const [z,c] of [[-.034,0xe32c24],[.034,0x32e974]])mesh(new THREE.SphereGeometry(.004,8,6),new THREE.MeshBasicMaterial({color:c}),.221,.242,z);
  const pts=[[-.13,.29,-.14],[-.18,.23,-.15],[.12,.21,-.15],[.16,.29,-.13]].map(p=>new THREE.Vector3(...p));
  mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts),28,.009,6,false),rubber,0,0,0);
  return g;
}

function huertaLeafTexture(kind,damaged){
  gl.hojasHuerta=gl.hojasHuerta||{};const key=kind+damaged;if(gl.hojasHuerta[key])return gl.hojasHuerta[key];
  const cv=document.createElement('canvas');cv.width=cv.height=256;const c=cv.getContext('2d');
  const hues={col:['#8aa993','#4f7764'],lechuga:['#a5bd5e','#587539'],papa:['#709153','#3e6035']};
  const grad=c.createLinearGradient(0,0,256,256);grad.addColorStop(0,hues[kind][0]);grad.addColorStop(1,hues[kind][1]);
  c.beginPath();
  for(let k=0;k<=100;k++){let a=k/100*Math.PI*2,r=1+(kind==='lechuga'?.08:.03)*Math.sin(a*13);let x=128+Math.cos(a)*100*r*Math.sin(a/2+.5)**2,y=128+Math.sin(a)*119*r;if(k===0)c.moveTo(x,y);else c.lineTo(x,y);}
  c.closePath();c.fillStyle=grad;c.fill();
  c.save();c.clip();c.strokeStyle=kind==='col'?'#bdd0a6':'#94ac62';c.lineWidth=3;c.beginPath();c.moveTo(128,250);c.quadraticCurveTo(119,130,128,12);c.stroke();
  for(let i=0;i<8;i++)for(const sign of [-1,1]){let y=38+i*25;c.lineWidth=1.1;c.beginPath();c.moveTo(124,y+27);c.quadraticCurveTo(128+sign*44,y+15,128+sign*95,y-15);c.stroke();}
  if(damaged){c.fillStyle='#92703e';for(let i=0;i<8;i++){c.beginPath();c.ellipse(50+(i*71)%155,38+(i*47)%170,9,13,i,0,Math.PI*2);c.fill();}c.globalCompositeOperation='destination-out';for(let i=0;i<5;i++){c.beginPath();c.ellipse(60+(i*43)%130,45+(i*59)%155,6,10,i,0,Math.PI*2);c.fill();}}
  c.restore();const tex=new THREE.CanvasTexture(cv);tex.anisotropy=4;gl.hojasHuerta[key]=tex;return tex;
}

function glPlanta(group,p,hLidar,detalle=1){
  const [x,y,r,damage,hh]=p,h=hh||.3;
  const kind=['col','lechuga','papa'][Math.abs(Math.round(x*100+y*173))%3];
  gl.hm=gl.hm||{};const key=kind+damage;
  if(!gl.hm[key])gl.hm[key]=new THREE.MeshStandardMaterial({map:huertaLeafTexture(kind,damage),side:THREE.DoubleSide,alphaTest:.45,roughness:.94});
  const leaves=kind==='papa'?12:10;
  for(let k=0;k<leaves;k++){
    const a=k*2.39996,inner=k>6;
    const len=kind==='papa'?h*.48:h*(inner?.65:1.05),width=kind==='papa'?r*.8:r*1.45;
    const geo=new THREE.PlaneGeometry(width,len,6,8);geo.translate(0,len/2,0);
    const pos=geo.attributes.position;
    for(let j=0;j<pos.count;j++){const u=pos.getX(j)/width,v=pos.getY(j)/len;pos.setZ(j,.05*Math.sin(v*Math.PI)*Math.cos(u*Math.PI)+.012*Math.sin(u*24+v*15)*v);}
    geo.computeVertexNormals();const leaf=new THREE.Mesh(geo,gl.hm[key]);
    leaf.position.set(x,kind==='papa'?k/leaves*h*.7:.025,-y);leaf.rotation.y=a;leaf.rotateX(inner?.3:(kind==='lechuga'?1.04:.8));leaf.castShadow=true;leaf.receiveShadow=true;group.add(leaf);
  }
  if(kind==='papa'){const stem=new THREE.Mesh(new THREE.CylinderGeometry(.007,.01,h,5),new THREE.MeshStandardMaterial({color:0x627746}));stem.position.set(x,h/2,-y);group.add(stem);}
}

const huertaOriginalWorld=glConstruirMundo;
glConstruirMundo=function(m){
  const prev=gl.mundoPintado;huertaOriginalWorld(m);if(prev===gl.mundoPintado||!gl.listo)return;
  // All added landmarks are visual approximations. Solid tank/stool footprints exist in world JSON.
  if(m.nombre!=='huerta_fotos_50cm')return;
  const earth=new THREE.MeshStandardMaterial({color:0x70614b,roughness:1});
  // Low soil mounds between rows, kept away from the actual 50 cm track.
  for(let i=0;i<3;i++){
    const bed=new THREE.Mesh(new THREE.SphereGeometry(1,24,10),earth);bed.scale.set(1.65,.09,.19);bed.position.set(0,-.025,(i-1)*.57);bed.receiveShadow=true;gl.grupoMundo.add(bed);
  }
  // Dry straw in deterministic clusters, one draw call.
  const points=[];for(let i=0;i<1700;i++){
    const x=Math.sin(i*13.71)*1.8,y=Math.sin(i*2.17)*.88,a=i*1.77;
    points.push(x,.045,-y,x+.055*Math.cos(a),.049,-y+.055*Math.sin(a));
  }
  const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(points,3));
  gl.grupoMundo.add(new THREE.LineSegments(geo,new THREE.LineBasicMaterial({color:0xb1a078})));
  const tank=new THREE.Mesh(new THREE.CylinderGeometry(.24,.23,.85,32),new THREE.MeshStandardMaterial({color:0x183b50,roughness:.55}));tank.position.set(-2.5,.425,-1.9);tank.castShadow=true;gl.grupoMundo.add(tank);
  const seat=new THREE.Mesh(new THREE.BoxGeometry(.26,.035,.26),new THREE.MeshStandardMaterial({color:0x276896}));seat.position.set(-2.5,.25,-1.35);seat.castShadow=true;gl.grupoMundo.add(seat);
  for(const dx of [-.10,.10])for(const dz of [-.10,.10]){const leg=new THREE.Mesh(new THREE.BoxGeometry(.035,.23,.035),seat.material);leg.position.set(-2.5+dx,.115,-1.35+dz);gl.grupoMundo.add(leg);}
  glLinealizar(gl.grupoMundo);
};
