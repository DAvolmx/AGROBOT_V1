const $ = id => document.getElementById(id);
const DESC = {R1: 'Delantera derecha', R2: 'Trasera derecha', L1: 'Delantera izquierda', L2: 'Trasera izquierda'};
let ruedas = [], nombres = ['R1', 'R2', 'L1', 'L2'], conectada = false, sosteniendo = null, timer = null;

const msg = t => { $('msg').textContent = t; };
async function api(url, body, metodo) {
  const r = await fetch(url, body === undefined ? {cache: 'no-store'} : {method: metodo || 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const d = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(d.error || ('error ' + r.status));
  return d;
}
function opciones(sel, valor) {
  sel.innerHTML = '';
  for (let p = 0; p <= 7; p++) { const o = document.createElement('option'); o.value = p; o.textContent = 'GPIO ' + p; sel.appendChild(o); }
  sel.value = valor;
}
function pintar() {
  $('estado').innerHTML = conectada ? '<span class="ok">C3 conectada</span>' : '<span class="bad">C3 sin enlace</span> (enciéndela y espera a que se una a JETSON_AP)';
  const tb = $('filas'); tb.innerHTML = '';
  ruedas.forEach(r => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td><b>${r.nombre}</b><br><span class="muted">${DESC[r.nombre] || ''}</span></td><td></td><td></td><td><input type="checkbox"></td><td><button class="sec">Guardar</button></td><td></td>`;
    const fi = document.createElement('select'), bi = document.createElement('select');
    opciones(fi, r.fi); opciones(bi, r.bi);
    tr.children[1].appendChild(fi); tr.children[2].appendChild(bi);
    const inv = tr.children[3].firstChild; inv.checked = r.invertida;
    tr.children[4].firstChild.onclick = async () => {
      try { await api('/api/c3/pines', {rueda: r.rueda, fi: +fi.value, bi: +bi.value, invertida: inv.checked}); msg(`${r.nombre} guardada en la C3.`); cargar(); }
      catch (e) { msg(e.message); cargar(); }
    };
    [['+', 1], ['−', -1]].forEach(([t, dir]) => {
      const b = document.createElement('button'); b.textContent = t; b.disabled = !conectada;
      b.onpointerdown = e => { e.preventDefault(); b.setPointerCapture(e.pointerId); probar(r.rueda, dir); };
      ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => b.addEventListener(ev, soltar));
      tr.children[5].appendChild(b);
    });
    tb.appendChild(tr);
  });
  [$('sa'), $('sb')].forEach((s, i) => { s.innerHTML = ''; nombres.forEach((n, k) => { const o = document.createElement('option'); o.value = k; o.textContent = n; s.appendChild(o); }); s.value = i; });
}
function enviarPrueba(rueda, sentido) {
  return api('/api/c3/prueba', {rueda, sentido, pct: sentido ? +$('pot').value : 0}).catch(e => msg(e.message));
}
function probar(rueda, dir) {
  soltar(); sosteniendo = {rueda, dir};
  enviarPrueba(rueda, dir); timer = setInterval(() => enviarPrueba(rueda, dir), 150);
}
function soltar() {
  if (timer) { clearInterval(timer); timer = null; }
  if (sosteniendo) { enviarPrueba(sosteniendo.rueda, 0); sosteniendo = null; }
}
async function cargar() {
  try { const d = await api('/api/c3/pines'); conectada = d.conectada; ruedas = d.ruedas; if (d.nombres) nombres = d.nombres; if (conectada && ruedas.length < 4) msg('La C3 respondió incompleta; pulsa Recargar.'); }
  catch (e) { conectada = false; msg(e.message); }
  pintar();
}
$('pot').oninput = () => { $('potv').textContent = $('pot').value + '%'; };
$('swap').onclick = async () => {
  try { await api('/api/c3/pines/intercambiar', {a: +$('sa').value, b: +$('sb').value}); msg('Pines intercambiados.'); } catch (e) { msg(e.message); }
  const LIM_TXT = {adel: 'Máx. adelante %', atras: 'Máx. atrás %', pivote: 'Potencia de pivote (traseras) %',
  minp: 'Arranque mínimo %', acel: 'Aceleración %/s', frena: 'Frenado %/s', tras: 'Fuerza extra de las traseras %',
  dif_del: '% del giro que hacen las delanteras (0 = solo traseras, 100 = skid-steer)', periodo: 'Ráfaga de tensión completa, ms (0 = PWM continuo)', refuerzo: 'Anti-hundimiento % (100 = apagado)', freno_ms: 'Freno activo al parar, ms (0 = rueda libre)',
  comp_neg: 'Giro a la IZQUIERDA: % que se apagan las ruedas que retroceden', comp_pos: 'Giro a la DERECHA: % que se apagan las ruedas que retroceden'};
async function cargarLimites() {
  try {
    const d = await api('/api/c3/limites'); const box = $('limites'); box.innerHTML = '';
    Object.entries(d.rangos).forEach(([k, r]) => {
      const lab = document.createElement('label'); lab.className = 'muted';
      lab.innerHTML = `${LIM_TXT[k] || k}<br><input type="number" min="${r.min}" max="${r.max}" step="1" style="width:90px">`
        + ` <button class="sec">Aplicar</button> <span class="muted">(${r.min}–${r.max})</span>`;
      const inp = lab.querySelector('input'); inp.value = d.limites[k] ?? r.defecto; inp.disabled = !d.conectada || d.limites[k] === undefined;
      lab.querySelector('button').disabled = inp.disabled;
      lab.querySelector('button').onclick = async () => {
        try { await api('/api/c3/limites', {nombre: k, valor: +inp.value}); msg(`${LIM_TXT[k]}: ${inp.value}. El robot está detenido.`); } catch (e) { msg(e.message); }
        cargarLimites();
      };
      box.appendChild(lab);
    });
  } catch (e) { msg(e.message); }
}
$('limReset').onclick = async () => { try { await api('/api/c3/limites/reset', {}); msg('Límites de fábrica restaurados.'); } catch (e) { msg(e.message); } cargarLimites(); };
cargar(); cargarLimites();
};
$('reset').onclick = async () => { try { await api('/api/c3/pines/reset', {}); msg('Valores de fábrica restaurados.'); } catch (e) { msg(e.message); } cargar(); };
$('recargar').onclick = cargar;
$('parar').onclick = () => { soltar(); fetch('/api/emergencia', {method: 'POST', keepalive: true}).catch(() => {}); };
window.addEventListener('blur', soltar);
document.addEventListener('visibilitychange', () => { if (document.hidden) soltar(); });
window.addEventListener('message', e => { if (e.origin === location.origin && e.data === 'motor-setup-stop') soltar(); });
cargar();
