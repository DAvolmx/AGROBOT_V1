const $=id=>document.getElementById(id);
const names=['Delantera derecha','Delantera izquierda','Trasera izquierda','Trasera derecha'];
const pins=[[0,1],[3,2],[4,5],[6,7]];
let selected=0,roles=[0,1,2,3],motors=[],dirty=false,compatible=false,verified=false,held=null,token='',seq=0,timer=null,inflight=false,generation=0;
function message(text){$('message').textContent=text}
async function post(action,data={}){const r=await fetch('/api/c3/setup/'+action,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),keepalive:action==='stop'});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);return d.value}
function render(){
 $('mapping').replaceChildren();roles.forEach((role,i)=>{const tr=document.createElement('tr');[`${i+1} · ${pins[i][0]}–${pins[i][1]}`,names[role],motors[i]?.invert?'Invertido':'Normal'].forEach(t=>{const td=document.createElement('td');td.textContent=t;tr.append(td)});$('mapping').append(tr)});
 document.querySelectorAll('[data-channel]').forEach(b=>b.classList.toggle('active',Number(b.dataset.channel)===selected));
 document.querySelectorAll('[data-role]').forEach(b=>{b.classList.toggle('selected',Number(b.dataset.role)===roles[selected]);b.disabled=!compatible});
 $('selected').textContent=`Canal ${selected+1}: GPIO ${pins[selected].join(' / ')} → ${names[roles[selected]]}`;
 $('dirty').textContent=dirty?'Cambios sin guardar':'Asignación cargada';
 ['forward','backward','invert','save','verify','jetson','applyServo'].forEach(id=>$(id).disabled=!compatible);
 $('verify').disabled=!compatible||dirty||!$('checked').checked;
 $('jetson').disabled=!compatible||dirty||!verified;
}
async function halt(){generation++;held=null;token='';clearInterval(timer);timer=null;try{await post('stop')}catch(e){message('Sin respuesta de C3. El firmware corta por timeout; usa el corte físico si es necesario.')}}
async function tick(){if(!held||!token||inflight)return;inflight=true;const h=held;try{await post('motor',{token,seq:seq++&65535,m:h.channel,d:h.dir,p:Number($('power').value)})}catch(e){message(e.message);await halt()}finally{inflight=false}}
async function begin(dir){if(!compatible||held)return;const mine=++generation;held={channel:selected,dir};seq=0;try{const t=await post('claim');if(mine!==generation||!held)return;token=t;await tick();if(mine===generation&&held)timer=setInterval(tick,120)}catch(e){message(e.message);await halt()}}
pins.forEach((pair,i)=>{const b=document.createElement('button');b.className='channel';b.dataset.channel=i;b.textContent=`Canal ${i+1} · GPIO ${pair.join(' / ')}`;b.onclick=async()=>{if(held)await halt();selected=i;render()};$('channels').append(b)});
document.querySelectorAll('[data-role]').forEach(b=>b.onclick=async()=>{if(held)await halt();const role=Number(b.dataset.role),other=roles.indexOf(role);[roles[selected],roles[other]]=[roles[other],roles[selected]];dirty=true;$('checked').checked=false;render()});
for(const [id,dir] of [['forward',1],['backward',-1]]){const b=$(id);b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);begin(dir)};['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>{if(held||token)halt()}));b.oncontextmenu=e=>e.preventDefault()}
$('stop').onclick=()=>{halt();fetch('/api/emergencia',{method:'POST',keepalive:true}).catch(()=>{})};
$('power').oninput=()=>{$('powerValue').textContent=$('power').value+'%'};
$('save').onclick=async()=>{await halt();try{await post('map',Object.fromEntries(roles.map((r,i)=>['r'+i,r])));dirty=false;$('checked').checked=false;message('Asignación guardada en C3. Verifica los sentidos antes de confirmar.');await refresh()}catch(e){message(e.message)}};
$('reload').onclick=()=>{dirty=false;refresh()};
$('invert').onclick=async()=>{await halt();try{await post('invert',{m:selected});$('checked').checked=false;message('Sentido invertido para este canal.');await refresh()}catch(e){message(e.message)}};
$('checked').onchange=render;
$('verify').onclick=async()=>{if(dirty||!$('checked').checked)return;await halt();try{await post('verify');message('Configuración confirmada.');await refresh()}catch(e){message(e.message)}};
$('jetson').onclick=async()=>{await halt();try{await post('jetson');message('Control entregado a Jetson; espera orden cero para armar.');await refresh()}catch(e){message(e.message)}};
function angle(a){$('servo').value=a;$('servoValue').textContent=a+'°'}
$('servo').oninput=()=>angle($('servo').value);document.querySelectorAll('[data-angle]').forEach(b=>b.onclick=()=>angle(b.dataset.angle));
$('applyServo').onclick=async()=>{await halt();try{const t=await post('claim');await post('actuator',{token:t,seq:0,servo:Number($('servo').value),led:0});message('Ángulo enviado: '+$('servo').value+'°.');await post('stop')}catch(e){message(e.message)}};
window.addEventListener('blur',()=>{if(held||token)halt()});document.addEventListener('visibilitychange',()=>{if(document.hidden&&(held||token))halt()});window.addEventListener('pagehide',()=>{if(held||token)navigator.sendBeacon('/api/c3/setup/stop',new Blob(['{}'],{type:'application/json'}))});
window.addEventListener('message',e=>{if(e.origin===location.origin&&e.data==='motor-setup-stop'&&(held||token))halt()});
async function refresh(){if(held||inflight)return;try{const r=await fetch('/api/c3/setup',{cache:'no-store'}),d=await r.json();if(!r.ok)throw Error(d.error||r.status);compatible=!!d.compatible;verified=!!d.verified;fillCfg(d);motors=d.motors||[];if(compatible&&!dirty&&motors.length===4)roles=motors.map(m=>m.role);$('status').textContent=compatible?`${d.firmware} · Control ${d.mode} · Configuración ${verified?'confirmada':'pendiente'} · ${d.fault?'FALLO PWM':'PWM disponible'}`:`Firmware ${d.firmware||'desconocido'}: carga demo-trasero-1.2 (o 1.1) para usar esta pestaña.`;if(d.fault)compatible=false;}catch(e){compatible=false;verified=false;$('status').textContent='C3 sin acceso: '+e.message}render()}
const CFG=['fwd','rev','minp','inner','slew','gap'];let cfgDirty=false;
CFG.forEach(k=>{const e=$('cfg_'+k);if(e)e.oninput=()=>{cfgDirty=true}});
function fillCfg(d){const ok=!!d.config_disponible&&!!d.cfg;['applyCfg','resetCfg'].forEach(id=>$(id).disabled=!ok);CFG.forEach(k=>{const e=$('cfg_'+k);if(e){e.disabled=!ok;if(ok&&!cfgDirty&&document.activeElement!==e)e.value=d.cfg[k]}});$('cfgNote').textContent=ok?'':'Requiere el firmware demo-trasero-1.2.'}
$('applyCfg').onclick=async()=>{await halt();try{const body={};CFG.forEach(k=>{body[k]=Number($('cfg_'+k).value)});await post('config',body);cfgDirty=false;message('Límites guardados en la C3. El robot está detenido.');await refresh()}catch(e){message(e.message)}};
$('resetCfg').onclick=async()=>{await halt();try{await post('config_reset');cfgDirty=false;message('Límites restaurados a los valores por defecto.');await refresh()}catch(e){message(e.message)}};
render();refresh();setInterval(refresh,2000);
