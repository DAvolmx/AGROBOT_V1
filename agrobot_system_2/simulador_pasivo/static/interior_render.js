// Visor de interiores (laboratorio, laberinto, almacén). Se carga tras el dashboard: envuelve glConstruirMundo y solo actúa en mundos «interior_*».
// Rectángulos cerrados de paredes pequeñas = muebles (bancos, mesas, estanterías): se dibujan como bloques macizos con tapa.
function glSepararMuebles(paredes) {
  const t = 0.03, H = [], V = [], libre = new Set();
  paredes.forEach((s, i) => {
    if (Math.abs(s[1] - s[3]) < t && Math.abs(s[0] - s[2]) > 0.2) H.push(i);
    else if (Math.abs(s[0] - s[2]) < t && Math.abs(s[1] - s[3]) > 0.2) V.push(i);
  });
  const muebles = [], usados = new Set();
  for (const a of H) for (const b of H) {
    if (a >= b || usados.has(a) || usados.has(b)) continue;
    const A = paredes[a], B = paredes[b];
    const ax0 = Math.min(A[0], A[2]), ax1 = Math.max(A[0], A[2]), bx0 = Math.min(B[0], B[2]), bx1 = Math.max(B[0], B[2]);
    if (Math.abs(ax0 - bx0) > t || Math.abs(ax1 - bx1) > t) continue;
    const ya = Math.min(A[1], B[1]), yb = Math.max(A[1], B[1]);
    if (yb - ya < 0.2 || yb - ya > 2.6 || ax1 - ax0 > 2.6) continue;
    const lados = V.filter((v) => {
      const S = paredes[v];
      return !usados.has(v) && (Math.abs(S[0] - ax0) < t || Math.abs(S[0] - ax1) < t) &&
             Math.min(S[1], S[3]) < ya + t && Math.max(S[1], S[3]) > yb - t;
    });
    if (lados.length < 2) continue;
    muebles.push([ax0, ya, ax1, yb]);
    usados.add(a); usados.add(b); lados.slice(0, 2).forEach((v) => usados.add(v));
  }
  const muros = paredes.filter((_, i) => !usados.has(i));
  return { muros, muebles };
}

function glTexturaBaldosa() {
  const cv = document.createElement("canvas"); cv.width = cv.height = 256;
  const c = cv.getContext("2d");
  c.fillStyle = "#b9b6ae"; c.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) {
    const g = 168 + ((i + j) % 2) * 14;
    c.fillStyle = `rgb(${g + 8},${g + 6},${g})`; c.fillRect(i * 128 + 2, j * 128 + 2, 124, 124);
  }
  for (let k = 0; k < 900; k++) { c.fillStyle = `rgba(80,76,70,${Math.random() * 0.05})`; c.fillRect(Math.random() * 256, Math.random() * 256, 2, 2); }
  const tex = new THREE.CanvasTexture(cv);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping; tex.anisotropy = 4;
  return tex;
}

const interiorOriginal = glConstruirMundo;
glConstruirMundo = function (m) {
  if (!m || !String(m.nombre || "").startsWith("interior_")) return interiorOriginal(m);
  const { muros, muebles } = glSepararMuebles(m.paredes);
  const m2 = Object.assign({}, m, { paredes: muros, altura_pared: 1.25 });
  const antes = gl.mundoPintado;
  interiorOriginal(m2);
  if (antes === gl.mundoPintado || !gl.listo) return;
  const [x0, y0, x1, y1] = m.limites, g = gl.grupoMundo;
  // suelo de baldosa 50 cm sobre el terreno
  const tb = glTexturaBaldosa(); tb.repeat.set((x1 - x0) / 1.0, (y1 - y0) / 1.0);
  const suelo = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), new THREE.MeshStandardMaterial({ map: tb, roughness: 0.55, metalness: 0.05 }));
  suelo.rotation.x = -Math.PI / 2; suelo.position.set((x0 + x1) / 2, 0.003, -(y0 + y1) / 2); suelo.receiveShadow = true; g.add(suelo);
  // muros: pintura clara en vez del bloque de huerta, con rodapié oscuro
  const yeso = new THREE.MeshStandardMaterial({ color: 0xe6e1d4, roughness: 0.92 });
  g.children.forEach((o) => {
    if (o.geometry && o.geometry.type === "BoxGeometry" && o.geometry.parameters.depth === 0.07) o.material = yeso;
  });
  const rodapie = new THREE.MeshStandardMaterial({ color: 0x4a4540, roughness: 0.7 });
  muros.forEach(([ax, ay, bx, by]) => {
    const L = Math.hypot(bx - ax, by - ay); if (L < 0.05) return;
    const r = new THREE.Mesh(new THREE.BoxGeometry(L, 0.09, 0.09), rodapie);
    r.position.set((ax + bx) / 2, 0.045, -(ay + by) / 2); r.rotation.y = Math.atan2(by - ay, bx - ax); r.receiveShadow = true; g.add(r);
  });
  // muebles macizos: tapa clara, cuerpo de melamina, patas oscuras
  const tapa = new THREE.MeshStandardMaterial({ color: 0xd8d2c4, roughness: 0.4, metalness: 0.05 });
  const cuerpo = new THREE.MeshStandardMaterial({ color: 0x7d8a93, roughness: 0.6 });
  muebles.forEach(([ax, ay, bx, by]) => {
    const w = bx - ax + 0.04, d = by - ay + 0.04, alto = Math.min(w, d) < 0.5 && Math.max(w, d) > 1.0 ? 1.6 : 0.78;   // estrecho y largo = estantería
    const b = new THREE.Mesh(new THREE.BoxGeometry(w, alto - 0.04, d), cuerpo);
    b.position.set((ax + bx) / 2, (alto - 0.04) / 2, -(ay + by) / 2); b.castShadow = b.receiveShadow = true; g.add(b);
    const t = new THREE.Mesh(new THREE.BoxGeometry(w + 0.02, 0.04, d + 0.02), tapa);
    t.position.set((ax + bx) / 2, alto - 0.02, -(ay + by) / 2); t.castShadow = t.receiveShadow = true; g.add(t);
  });
  // luminarias de techo (paneles emisivos sobre el recorrido; no proyectan luz extra para no cargar el visor)
  const panel = new THREE.MeshBasicMaterial({ color: 0xfff6dc });
  const nx = Math.max(1, Math.round((x1 - x0) / 2.2)), ny = Math.max(1, Math.round((y1 - y0) / 2.2));
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
    const p = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.03, 0.3), panel);
    p.position.set(x0 + (i + 0.5) * (x1 - x0) / nx, 2.55, -(y0 + (j + 0.5) * (y1 - y0) / ny)); g.add(p);
  }
  // meta: baliza verde para ver adónde va el recorrido
  if (m.meta) {
    const baliza = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.02, 24), new THREE.MeshBasicMaterial({ color: 0x38c172 }));
    baliza.position.set(m.meta[0], 0.012, -m.meta[1]); g.add(baliza);
  }
};
