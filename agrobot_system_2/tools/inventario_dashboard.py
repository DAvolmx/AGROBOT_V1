"""Inventario de funcionalidades del dashboard para el informe de la tesis: rutas REST del backend (con la primera frase del docstring del manejador), mensajes WebSocket
(emitidos por el backend y manejados por el dashboard), URL que usa cada control y estadísticas del código. Salida: docs/INFORME_DASHBOARD/inventario.json.
Uso: python3 tools/inventario_dashboard.py"""
import ast, json, re
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
OUT = RAIZ / "docs" / "INFORME_DASHBOARD"
OUT.mkdir(parents=True, exist_ok=True)
SRV = (RAIZ / "jetson" / "backend" / "jetson_server.py").read_text(encoding="utf-8")
HTML = (RAIZ / "jetson" / "dashboard" / "index.html").read_text(encoding="utf-8")

# --- rutas REST + docstring del manejador (jetson_server.py y los módulos que registran las suyas)
FUENTES = ["jetson_server.py", "recoleccion.py", "etiquetado.py"]
PAGINAS = {p.name: p.read_text(encoding="utf-8") for p in (RAIZ / "jetson" / "dashboard").glob("*.html") if ".bak" not in p.name and ".backup" not in p.name}
rutas = []
for fuente in FUENTES:
    txt = (RAIZ / "jetson" / "backend" / fuente).read_text(encoding="utf-8")
    doc = {}
    for n in ast.walk(ast.parse(txt)):
        if isinstance(n, (ast.AsyncFunctionDef, ast.FunctionDef)):
            d = ast.get_docstring(n) or ""
            doc[n.name] = re.sub(r"\s+", " ", d.split("\n\n")[0]).strip()
    for m in re.finditer(r"""app\.router\.add_(get|post|put|delete|route|static)\(\s*(?:["'][A-Z*]+["'],\s*)?["']([^"']+)["']\s*,\s*([A-Za-z_0-9\.]+)""", txt):
        metodo, ruta, h = m.groups()
        rutas.append({"metodo": metodo.upper(), "ruta": ruta, "manejador": h, "modulo": fuente, "doc": doc.get(h.split(".")[-1], "")[:220]})
for r in rutas:
    base = re.sub(r"\{[^}]+\}", "", r["ruta"]).rstrip("/")
    usos = []
    for pag, txt in PAGINAS.items():
        urls = set(re.findall(r"""["'`](/[A-Za-z0-9_/\-\.]+)""", txt))
        if any(u.rstrip("/") == base or (base.startswith("/api/") and (u.rstrip("/") + "/").startswith(base + "/")) for u in urls):
            usos.append(pag)
    r["usada_por"] = usos
    r["usada_por_dashboard"] = bool(usos)

# --- mensajes WebSocket: lo que el dashboard maneja y lo que el backend emite
maneja = sorted(set(re.findall(r'case "([a-z_]+)":', HTML)))
emite = sorted(set(re.findall(r'"type":\s*"([a-z_]+)"', SRV)))

# --- URL por control: ventana tras $("id").addEventListener(
ctrl = {}
for m in re.finditer(r'\$\("([a-zA-Z0-9\-_]+)"\)\.addEventListener\("(click|input|change)"', HTML):
    cid, ev = m.groups()
    ini = m.end()
    sig = re.search(r'\$\("[a-zA-Z0-9\-_]+"\)\.addEventListener\(', HTML[ini:])
    ventana = HTML[ini: ini + (sig.start() if sig else 1500)][:1800]
    urls = sorted(set(re.findall(r"""["'`](/api/[A-Za-z0-9_/\-\.]+)""", ventana)))
    post = bool(re.search(r'method:\s*"(POST|PUT|DELETE)"', ventana))
    ctrl.setdefault(cid, []).append({"evento": ev, "urls": urls, "escribe": post})

stats = {"lineas_index_html": HTML.count("\n") + 1, "tamano_kb": round(len(HTML.encode()) / 1024), "botones": len(re.findall(r"<button", HTML)), "deslizadores": len(re.findall(r'type="range"', HTML)),
         "interruptores": len(re.findall(r'class="switch"', HTML)), "lienzos": len(re.findall(r"<canvas", HTML)), "ids": len(re.findall(r'\bid="', HTML)),
         "rutas_rest_backend": len(rutas), "rutas_usadas_por_el_dashboard": sum(1 for r in rutas if "index.html" in r["usada_por"]), "rutas_usadas_por_alguna_pagina": sum(1 for r in rutas if r["usada_por"]), "mensajes_ws_manejados": len(maneja), "mensajes_ws_emitidos": len(emite),
         "pestanas": len(re.findall(r'data-tab="tab-', HTML)) // 1, "lineas_jetson_server_py": SRV.count("\n") + 1}
# calidad del código del dashboard
rutas_por_grupo = {}
for r in rutas:
    g = (r["ruta"].split("/")[2] if r["ruta"].startswith("/api/") and len(r["ruta"].split("/")) > 2 else r["ruta"].strip("/") or "raíz")
    rutas_por_grupo[g] = rutas_por_grupo.get(g, 0) + 1
stats.update({"aria_label": HTML.count("aria-label="), "atributos_title": len(re.findall(r'\btitle="', HTML)), "media_queries": HTML.count("@media"),
              "confirmaciones_confirm": len(re.findall(r"\bconfirm\(", HTML)), "usos_localstorage": HTML.count("localStorage"), "intervalos_setInterval": len(re.findall(r"setInterval\(", HTML)),
              "llamadas_fetch": len(re.findall(r"\bfetch\(", HTML)), "reconexion_ws": bool(re.search(r"onclose\s*=\s*\(\)\s*=>\s*\{[^}]*setTimeout\(connect", HTML)),
              "autenticacion_basic_opcional": "exigir_clave" in SRV, "clave_configurada": (Path.home() / ".config" / "agrobot" / "clave_dashboard").exists(), "paginas_auxiliares": 4})
(OUT / "inventario.json").write_text(json.dumps({"stats": stats, "rutas_por_grupo": rutas_por_grupo, "rutas": rutas, "ws_maneja": maneja, "ws_emite": emite, "controles": ctrl}, indent=1, ensure_ascii=False))
print(json.dumps(stats, indent=1))
print("WS maneja:", maneja)
print("WS emite:", emite)
