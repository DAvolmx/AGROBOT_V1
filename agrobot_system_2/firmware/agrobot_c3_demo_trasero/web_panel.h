#pragma once
const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AGROBOT C3</title>
<style>*{box-sizing:border-box}body{font:15px system-ui;max-width:780px;margin:auto;padding:16px;background:#eff6f1;color:#173627}h1{font-size:23px}.card{border:1px solid #bccfc2;background:white;border-radius:12px;padding:16px;margin:12px 0}button{padding:14px;border:0;border-radius:8px;margin:4px;background:#196343;color:white;touch-action:none;cursor:pointer}.stop{background:#af2735;font-weight:bold;min-width:180px}.row{display:flex;flex-wrap:wrap;gap:6px}input{max-width:100%}pre{white-space:pre-wrap;font-size:13px}.note{font-size:13px;color:#536959}</style>
<h1>AGROBOT · C3 demo trasero 1.1</h1><p class="note">Cliente JETSON_AP · UDP + web, sin BLE. Pivote trasero libre. Identificación con ruedas elevadas; sin movimiento al encender.</p>
<button class="stop" id="stop">PARADA / bloquear órdenes Jetson</button><button id="jetson">Entregar control a Jetson</button>
<div class="card"><b>Identificar motores</b><p class="note">Un motor por vez. + significa avance esperado. Si gira al revés, invertir. Los nombres R/L deben coincidir con el lado físico; corregir conexiones si están intercambiados.</p>
<label>PWM de prueba <input id="testPower" type="range" min="0" max="50" value="25"><span id="testValue">25</span>%</label><div id="motors"></div>
<button id="verify">Confirmar que los 4 motores avanzan correctamente</button></div>
<div class="card"><b>Conducción manual · mantener pulsado</b><p class="note">Requiere mapeo confirmado. Sin giro sobre el sitio. Límite 60% PWM adelante y 35% atrás. Retroceso recto. La orden no es velocidad medida.</p>
<label>Orden <input id="power" type="range" min="0" max="60" value="25"><span id="powerValue">25</span>% del límite</label>
<div class="row"><button data-drive="1,1">Avanzar izquierda</button><button data-drive="1,0">Adelante</button><button data-drive="1,-1">Avanzar derecha</button><button data-drive="-1,0">Atrás</button></div></div>
<div class="card"><b>Cámara y foco</b><p class="note">Servo de inclinación de cámara, no dirección. Menor ángulo = más abajo según calibración provisional. El foco solo permanece encendido al mantener el botón.</p>
<label>Servo <input id="servo" type="range" min="25" max="130" value="98"><span id="servoValue">98</span>°</label><button id="servoApply">Aplicar cámara</button><button id="light" data-light="1">Mantener foco encendido</button></div>
<div class="card"><b>Diagnóstico</b><pre id="state">Consultando…</pre><p class="note">Red Jetson: 192.168.12.240. La C3 no crea AP ni alterna redes. Clave de red 12345678. Web: agrobot / 12345678.</p></div>
<script>
let held=null,token='',seq=0,timer=null,inflight=false;const state=document.getElementById('state');
const $=id=>document.getElementById(id);
async function post(path,args={}){const r=await fetch(path,{method:'POST',body:new URLSearchParams(args),cache:'no-store'});const t=await r.text();if(!r.ok)throw Error(t);return t}
async function halt(){held=null;token='';clearInterval(timer);timer=null;try{await post('/stop')}catch(e){state.textContent='Sin enlace; parada por timeout en C3'}}
async function tick(){if(!held||!token||inflight)return;inflight=true;const h=held;
try{let args={token,seq:seq++&65535},path;
 if(h.dataset.motor){path='/motor';args.m=h.dataset.motor;args.d=h.dataset.dir;args.p=$('testPower').value}
 else if(h.dataset.drive){path='/drive';let [v,w]=h.dataset.drive.split(',').map(Number);args.v=v*Number($('power').value);args.w=w*100}
 else{path='/actuator';args.servo=$('servo').value;args.led=1}
 await post(path,args)
}catch(e){state.textContent=e.message;await halt()}finally{inflight=false}}
async function begin(b){if(held)return;held=b;const original=b;seq=0;
try{const t=await post('/claim');if(held!==original)return;token=t;await tick();if(held)timer=setInterval(tick,150)}catch(e){state.textContent=e.message;await halt()}}
const wheelNames=["delantera derecha","delantera izquierda","trasera izquierda","trasera derecha"];
const names=['R1 delantero derecho','L1 delantero izquierdo','L2 trasero izquierdo','R2 trasero derecho'];
names.forEach((name,i)=>$('motors').insertAdjacentHTML('beforeend',`<div class="row"><b style="width:200px">${name}</b><button data-motor="${i}" data-dir="1">Mantener +</button><button data-motor="${i}" data-dir="-1">Mantener −</button><button data-invert="${i}">Invertir sentido</button></div>`));
document.querySelectorAll('[data-motor],[data-drive],[data-light]').forEach(b=>{
 b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);begin(b)};
 ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>halt()));
});
document.querySelectorAll('[data-invert]').forEach(b=>b.onclick=async()=>{await halt();try{state.textContent=await post('/invert',{m:b.dataset.invert});await status()}catch(e){state.textContent=e.message}});
$('stop').onclick=halt;
$('jetson').onclick=async()=>{await halt();try{state.textContent=await post('/jetson')}catch(e){state.textContent=e.message}};
$('verify').onclick=async()=>{await halt();try{state.textContent=await post('/verify');await status()}catch(e){state.textContent=e.message}};
$('servoApply').onclick=async()=>{await halt();try{let t=await post('/claim');await post('/actuator',{token:t,seq:0,servo:$('servo').value,led:0})}catch(e){state.textContent=e.message}};
['power','testPower','servo'].forEach(id=>$(id).oninput=()=>$(id==='testPower'?'testValue':id+'Value').textContent=$(id).value);
document.addEventListener('visibilitychange',()=>{if(document.hidden&&(held||token))halt()});window.addEventListener('blur',()=>{if(held||token)halt()});window.addEventListener('pagehide',()=>{if(held||token)navigator.sendBeacon('/stop','')});
async function status(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);const d=await r.json();
state.textContent=`Firmware: ${d.firmware} · Control: ${d.mode} · mapeo: ${d.verified?'confirmado':'PENDIENTE'} · Jetson armada: ${d.armed}\nIP STA: ${d.ip} · AP respaldo: ${d.ap?'192.168.4.1':'apagado'}\nRAM libre: ${d.heap} bytes · timeout: ${d.timeout} · fallo PWM: ${d.fault}\nEdad Jetson: ${d.jetson_age_ms===4294967295?'sin órdenes':d.jetson_age_ms+' ms'} · paquetes rechazados: ${d.bad_packets}\n`+d.motors.map(m=>`Canal ${m.name} -> ${wheelNames[m.role]} FI=${m.fi} BI=${m.bi} inversión=${m.invert} PWM=${m.duty.toFixed(1)}%`).join('\n')
}catch(e){state.textContent='Sin conexión: '+e.message}}
status();setInterval(status,2000);
</script></html>
)HTML";
