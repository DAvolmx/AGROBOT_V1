# AGROBOT · Corrección de SLAM 2D (odometría por scans) — UNA SOLA CELDA de Kaggle: entrena, evalúa, grafica y exporta.
# Dataset esperado: /kaggle/input/agrobot-slam-correction/dataset.npz (lo genera simulador_pasivo/generar_dataset_slam.py).
# Se entrena con el error del INCREMENTO (observable en la ventana K=5); la evaluación se hace sobre la pose integrada.
import json, math, os, pickle, random, time
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import matplotlib
try:
    get_ipython()                      # en Kaggle/Jupyter las gráficas salen en línea
except NameError:
    matplotlib.use("Agg")              # fuera de un notebook (pruebas locales)
import matplotlib.pyplot as plt
import pandas as pd
pd.set_option('display.width', 220); pd.set_option('display.max_columns', None)
try:
    from IPython.display import display
except Exception:
    display = print
def tabla(titulo, df):
    print(f"\n=== {titulo} ===")
    display(df)

SEMILLA = 0
random.seed(SEMILLA); np.random.seed(SEMILLA); torch.manual_seed(SEMILLA); torch.cuda.manual_seed_all(SEMILLA)
torch.backends.cudnn.deterministic = True
DEVICE = "cuda" if (torch.cuda.is_available() and not os.environ.get("AGROBOT_CPU")) else "cpu"
SALIDA = Path(os.environ.get("AGROBOT_SALIDA", "/kaggle/working")); SALIDA.mkdir(parents=True, exist_ok=True)
RUTA = Path(os.environ.get("AGROBOT_NPZ", "/kaggle/input/agrobot-slam-correction/dataset.npz"))
if not RUTA.exists():                                  # Kaggle monta los datasets en rutas distintas según la versión: se busca
    hallados = sorted(Path("/kaggle/input").rglob("dataset.npz")) if Path("/kaggle/input").exists() else []
    if hallados:
        RUTA = hallados[0]
if not RUTA.exists():
    contenido = [str(p) for p in Path("/kaggle/input").rglob("*")][:40] if Path("/kaggle/input").exists() else []
    raise FileNotFoundError(f"Falta el dataset: se esperaba {RUTA}. Genera dataset.npz con el simulador (generar_dataset_slam.py) y "
                            f"súbelo como Dataset privado de Kaggle ('agrobot-slam-correction') y añádelo con 'Add Input'. "
                            f"Contenido de /kaggle/input: {contenido}")
print("dispositivo:", DEVICE, "· dataset:", RUTA)

d = np.load(RUTA, allow_pickle=False)
X, Y_ACUM, Y_INC = d["features"].astype(np.float32), d["targets"].astype(np.float32), d["targets_inc"].astype(np.float32)
ESC, FALLA, SEM, EP = d["escenario"].astype(str), d["falla"].astype(str), d["semilla"], d["episodio_id"]
EST, REAL, T = d["est_pose"].astype(np.float64), d["real_pose"].astype(np.float64), d["timestamp"]
K = int(d["K"]); NOMBRES = list(d["nombres_features"]); NB = len(NOMBRES)
D_ESPERADO = K * NB
assert X.shape[1] == D_ESPERADO, f"D={X.shape[1]} no coincide con K*{NB}={D_ESPERADO}"
assert X.shape[0] == Y_ACUM.shape[0] == Y_INC.shape[0] == len(ESC)
print("meta:", json.loads(str(d["meta"])))
print(f"N={len(X)} · D={X.shape[1]} (K={K} x {NB}) · episodios={len(np.unique(EP))}")
bad = ~np.isfinite(X).all(1) | ~np.isfinite(Y_INC).all(1)
print("filas con NaN/inf:", int(bad.sum()))
X = np.nan_to_num(X, nan=0.0, posinf=8.0, neginf=0.0)
print("rango targets (acumulado) min/max:", Y_ACUM.min(0).round(3), Y_ACUM.max(0).round(3))
print("rango targets_inc min/max:", Y_INC.min(0).round(4), Y_INC.max(0).round(4))
print("% scans NO válidos (scan_ok=0, fotograma actual):", round(100 * float((X[:, (K - 1) * NB + NOMBRES.index('scan_ok')] < 0.5).mean()), 2))
for nom, arr in (("escenario", ESC), ("falla", FALLA)):
    u, c = np.unique(arr, return_counts=True)
    print(nom, dict(zip(u, c)))
corr = np.nan_to_num(np.corrcoef(np.c_[X[:, (K - 1) * NB:], Y_INC].T)[:NB, NB:])
print("correlación |features (fotograma actual)| vs targets_inc [dx, dy, dth]:")
for n, c in zip(NOMBRES, corr):
    print(f"  {n:13s} {np.round(c, 3)}")

TODOS = sorted(set(ESC))
VAL_ESC = os.environ.get("AGROBOT_VAL_ESC", "curva_pasillo,hileras_cuadricula").split(",")
faltan = [e for e in VAL_ESC if e not in TODOS]
if faltan:                                            # dataset pequeño/parcial: se eligen escenarios de prueba entre los que hay
    VAL_ESC = TODOS[-min(2, len(TODOS) - 1):]
    print(f"AVISO: faltan {faltan} en el dataset (hay {TODOS}); la prueba usará {VAL_ESC}")
assert len(TODOS) >= 2, "el dataset necesita al menos 2 escenarios (uno de train y uno de test)"
TRAIN_ESC = [e for e in TODOS if e not in VAL_ESC]
sem_train = sorted(set(SEM[np.isin(ESC, TRAIN_ESC)]))
sem_dev = sem_train[-2:] if len(sem_train) > 2 else sem_train[-1:]
m_test = np.isin(ESC, VAL_ESC)
m_dev = np.isin(ESC, TRAIN_ESC) & np.isin(SEM, sem_dev)
m_tr = np.isin(ESC, TRAIN_ESC) & ~np.isin(SEM, sem_dev)
print("train:", TRAIN_ESC, "semillas", [s for s in sem_train if s not in sem_dev])
print("dev  :", TRAIN_ESC, "semillas", sem_dev)
print("test :", VAL_ESC, "(NO vistos)")
tabla("Split", pd.DataFrame({"conjunto": ["train", "dev (early stopping)", "test (NO visto)"], "escenarios": [", ".join(TRAIN_ESC), ", ".join(TRAIN_ESC), ", ".join(VAL_ESC)], "muestras": [int(m_tr.sum()), int(m_dev.sum()), int(m_test.sum())]}))
assert m_tr.sum() > 0 and m_dev.sum() > 0 and m_test.sum() > 0, "split vacío: el dataset necesita >=3 semillas y escenarios de train y de test"

mu_x, sd_x = X[m_tr].mean(0), X[m_tr].std(0) + 1e-6
mu_y, sd_y = Y_INC[m_tr].mean(0), Y_INC[m_tr].std(0) + 1e-9
nx = lambda A: ((A - mu_x) / sd_x).astype(np.float32)
ny = lambda A: ((A - mu_y) / sd_y).astype(np.float32)
with open(SALIDA / "scaler.pkl", "wb") as f:      # dict simple (sin sklearn) para que el robot lo cargue con NumPy
    pickle.dump({"mean": mu_x, "scale": sd_x, "y_mean": mu_y, "y_scale": sd_y, "K": K, "nombres": NOMBRES}, f)
tt = lambda a: torch.tensor(a, device=DEVICE)
Xtr, Ytr, Xdv, Ydv = tt(nx(X[m_tr])), tt(ny(Y_INC[m_tr])), tt(nx(X[m_dev])), tt(ny(Y_INC[m_dev]))

class MLP(nn.Module):
    def __init__(self, d_in, d_out=3):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(d_in, 128), nn.ReLU(), nn.Dropout(0.1),
                                 nn.Linear(128, 64), nn.ReLU(), nn.Dropout(0.1),
                                 nn.Linear(64, 32), nn.ReLU(), nn.Linear(32, d_out))
    def forward(self, x): return self.net(x)

modelo = MLP(D_ESPERADO).to(DEVICE)
n_par = sum(p.numel() for p in modelo.parameters())
print("parámetros:", n_par)
assert n_par < 100_000

opt = torch.optim.Adam(modelo.parameters(), lr=1e-3)
perdida = nn.HuberLoss(delta=1.0)
g = torch.Generator().manual_seed(SEMILLA)
mejor, mala, hist = math.inf, 0, []
t0 = time.time()
for ep in range(150):
    modelo.train()
    perm = torch.randperm(len(Xtr), generator=g).to(DEVICE)
    tot = 0.0
    for i in range(0, len(perm), 64):
        b = perm[i:i + 64]
        opt.zero_grad()
        l = perdida(modelo(Xtr[b]), Ytr[b]); l.backward(); opt.step()
        tot += float(l) * len(b)
    modelo.eval()
    with torch.no_grad():
        lv = float(perdida(modelo(Xdv), Ydv))
    hist.append((tot / len(Xtr), lv))
    if lv < mejor - 1e-5:
        mejor, mala = lv, 0
        torch.save(modelo.state_dict(), SALIDA / "best_model.pth")
    else:
        mala += 1
    if ep % 10 == 0:
        print(f"época {ep:3d} · train {hist[-1][0]:.4f} · dev {lv:.4f} · mejor {mejor:.4f}")
    if mala >= 15:
        print(f"parada temprana en la época {ep}"); break
modelo.load_state_dict(torch.load(SALIDA / "best_model.pth", map_location=DEVICE)); modelo.eval()
print(f"entrenado en {time.time() - t0:.0f} s · mejor dev {mejor:.4f}")

def componer(p, inc):
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([p[0] + c * inc[0] - s * inc[1], p[1] + s * inc[0] + c * inc[1], (p[2] + inc[2] + math.pi) % (2 * math.pi) - math.pi])

def relativa(pj, pk):
    c, s = math.cos(pj[2]), math.sin(pj[2]); dx, dy = pk[0] - pj[0], pk[1] - pj[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, (pk[2] - pj[2] + math.pi) % (2 * math.pi) - math.pi])

def predecir(Xe):
    with torch.no_grad():
        p = modelo(torch.tensor(nx(Xe), device=next(modelo.parameters()).device)).cpu().numpy()
    return p * sd_y + mu_y

def ate(est, real):
    a, b = est[:, :2], real[:, :2]
    ma, mb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    e = (a - ma) @ R.T + mb - b
    return float(np.sqrt((e ** 2).sum(1).mean()))

def rpe(est, real, paso=1.0):
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(real[:, :2], axis=0).T))]
    et, er = [], []
    for i in range(len(real)):
        j = np.searchsorted(s, s[i] + paso)
        if j >= len(real): break
        dr, de = relativa(real[i], real[j]), relativa(est[i], est[j])
        et.append(math.hypot(*(dr[:2] - de[:2]))); er.append(abs(math.degrees((dr[2] - de[2] + math.pi) % (2 * math.pi) - math.pi)))
    return (float(np.mean(et)) if et else float("nan")), (float(np.mean(er)) if er else float("nan"))

filas = []
trayectorias = {}
for e in np.unique(EP):
    m = EP == e
    if m.sum() < 10: continue
    est_b, real, xe = EST[m], REAL[m], X[m]
    inc_b = np.array([relativa(est_b[i - 1], est_b[i]) for i in range(1, len(est_b))])
    corr = predecir(xe)[1:]
    est_c = [est_b[0].copy()]
    for inc, c in zip(inc_b, corr): est_c.append(componer(est_c[-1], inc + c))
    est_c = np.array(est_c)
    rb, rc = rpe(est_b, real), rpe(est_c, real)
    fin = lambda es: float(np.hypot(*(es[-1, :2] - real[-1, :2])))
    filas.append(dict(escenario=ESC[m][0], falla=FALLA[m][0], semilla=int(SEM[m][0]), conjunto="test" if ESC[m][0] in VAL_ESC else "train",
                      ATE_base=ate(est_b, real), ATE_corr=ate(est_c, real), RPE_base=rb[0], RPE_corr=rc[0], RPEgrados_base=rb[1],
                      RPEgrados_corr=rc[1], cierre_base=fin(est_b), cierre_corr=fin(est_c)))
    trayectorias[int(e)] = (real, est_b, est_c)
import csv
with open(SALIDA / "metricas.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(filas[0].keys())); w.writeheader(); w.writerows(filas)
def media(conj, k): return float(np.nanmean([r[k] for r in filas if r["conjunto"] == conj]))
# ---------------- TABLAS ----------------
df = pd.DataFrame(filas)
resumen = []
for conj in ("train", "test"):
    sub = df[df.conjunto == conj]
    if sub.empty: continue
    for k, u, f in (("ATE", "cm", 100), ("RPE", "cm (1 m)", 100), ("RPEgrados", "° (1 m)", 1), ("cierre", "cm (error final)", 100)):
        b_, c_ = sub[k + "_base"].mean() * f, sub[k + "_corr"].mean() * f
        resumen.append({"conjunto": conj, "métrica": f"{k} [{u}]", "episodios": len(sub), "base": round(b_, 2), "corregido": round(c_, 2),
                        "mejora %": round(100 * (b_ - c_) / max(b_, 1e-9), 1)})
tabla("Resumen: línea base (sin corrección) vs corregido", pd.DataFrame(resumen))
for col, nombre in (("escenario", "por ESCENARIO"), ("falla", "por FALLA")):
    t = df.groupby([ "conjunto", col])[["ATE_base", "ATE_corr", "RPE_base", "RPE_corr", "cierre_base", "cierre_corr"]].mean().mul(100).round(1)
    t["mejora ATE %"] = (100 * (t.ATE_base - t.ATE_corr) / t.ATE_base.clip(lower=1e-9)).round(1)
    tabla(f"ATE / RPE / error final en cm, {nombre}", t.reset_index())

# ---------------- GRÁFICAS ----------------
def dibujar(ep, titulo, ax):
    real, b, c = trayectorias[ep]
    ax.plot(real[:, 0], real[:, 1], "k-", lw=2, label="real"); ax.plot(b[:, 0], b[:, 1], "r--", label="estimada (base)")
    ax.plot(c[:, 0], c[:, 1], "g-", label="corregida"); ax.set_aspect("equal"); ax.set_title(titulo, fontsize=9); ax.legend(fontsize=7); ax.grid(alpha=.3)
ep_conj = {int(e): ("test" if ESC[EP == e][0] in VAL_ESC else "train") for e in trayectorias}
# 1) EDA
fig, axs = plt.subplots(1, 4, figsize=(18, 3.8))
for j, n in enumerate(("dx", "dy", "dθ")):
    axs[j].hist(Y_INC[:, j], bins=80, color="#3b6ea5"); axs[j].set_title(f"error del incremento {n}"); axs[j].grid(alpha=.3)
u, c = np.unique(ESC, return_counts=True); axs[3].barh(u, c, color="#6a994e"); axs[3].set_title("muestras por escenario")
fig.tight_layout(); fig.savefig(SALIDA / "01_eda.png", dpi=110); plt.show()
# 2) pérdida
fig, ax = plt.subplots(figsize=(6, 3.6)); ax.plot([h[0] for h in hist], label="train"); ax.plot([h[1] for h in hist], label="dev")
ax.set_title("pérdida (Huber)"); ax.set_xlabel("época"); ax.legend(); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(SALIDA / "02_perdida.png", dpi=110); plt.show()
# 3) trayectorias: 2 de train y 2 de test (real vs base vs corregida)
elegidos = [e for e in trayectorias if ep_conj[e] == "train"][:2] + [e for e in trayectorias if ep_conj[e] == "test"][:2]
fig, axs = plt.subplots(1, len(elegidos), figsize=(5 * len(elegidos), 4.6), squeeze=False)
for ax, e in zip(axs[0], elegidos): dibujar(e, f"{ep_conj[e]}: {ESC[EP == e][0]} · {FALLA[EP == e][0]}", ax)
fig.tight_layout(); fig.savefig(SALIDA / "03_trayectorias.png", dpi=110); plt.show()
# 4) barras ATE base vs corregido (test) por escenario y por falla
sub = df[df.conjunto == "test"] if (df.conjunto == "test").any() else df
fig, axs = plt.subplots(1, 2, figsize=(15, 4))
for ax, col in zip(axs, ("escenario", "falla")):
    g_ = sub.groupby(col)[["ATE_base", "ATE_corr"]].mean() * 100
    g_.plot.bar(ax=ax, color=["#c0392b", "#27ae60"]); ax.axhline(10, color="k", ls=":", label="criterio 10 cm"); ax.set_ylabel("ATE [cm]")
    ax.set_title(f"ATE por {col} (conjunto {'test' if (df.conjunto == 'test').any() else 'train'})"); ax.legend(["criterio 10 cm", "base", "corregido"], fontsize=7); ax.grid(alpha=.3)
fig.tight_layout(); fig.savefig(SALIDA / "04_ate_barras.png", dpi=110); plt.show()
# 5) predicción vs real del error de incremento (test)
Xt = X[m_test]; Pt = predecir(Xt); Rt = Y_INC[m_test]
fig, axs = plt.subplots(1, 3, figsize=(15, 4))
for j, n in enumerate(("dx", "dy", "dθ")):
    axs[j].scatter(Rt[:, j], Pt[:, j], s=3, alpha=.3); lim = [min(Rt[:, j].min(), Pt[:, j].min()), max(Rt[:, j].max(), Pt[:, j].max())]
    axs[j].plot(lim, lim, "k--", lw=1); r2 = 1 - ((Rt[:, j] - Pt[:, j]) ** 2).sum() / max(((Rt[:, j] - Rt[:, j].mean()) ** 2).sum(), 1e-12)
    axs[j].set_title(f"{n}: predicho vs real (test) · R²={r2:.2f}"); axs[j].set_xlabel("real"); axs[j].set_ylabel("predicho"); axs[j].grid(alpha=.3)
fig.tight_layout(); fig.savefig(SALIDA / "05_prediccion.png", dpi=110); plt.show()
# 6) error de posición a lo largo del recorrido (media de los episodios del conjunto evaluado)
fig, ax = plt.subplots(figsize=(7, 4))
for etq, col in (("base", "#c0392b"), ("corregida", "#27ae60")):
    curvas = []
    for e, (real, b, c) in trayectorias.items():
        if ep_conj[e] != ("test" if (df.conjunto == "test").any() else "train"): continue
        s_ = np.r_[0, np.cumsum(np.hypot(*np.diff(real[:, :2], axis=0).T))]; est_ = b if etq == "base" else c
        curvas.append(np.interp(np.linspace(0, 4, 60), s_, np.hypot(*(est_[:, :2] - real[:, :2]).T)))
    if curvas: ax.plot(np.linspace(0, 4, 60), np.mean(curvas, 0) * 100, color=col, label=etq)
ax.set_xlabel("distancia recorrida [m]"); ax.set_ylabel("error de posición [cm]"); ax.set_title("deriva a lo largo del recorrido"); ax.legend(); ax.grid(alpha=.3)
fig.tight_layout(); fig.savefig(SALIDA / "06_deriva.png", dpi=110); plt.show()

# ---------------- DATOS REALES (opcional: real.npz junto a dataset.npz) ----------------
RUTA_REAL = RUTA.with_name("real.npz")
if RUTA_REAL.exists():
    r_ = np.load(RUTA_REAL, allow_pickle=False)
    XR = np.nan_to_num(r_["features"].astype(np.float32), nan=0.0, posinf=8.0, neginf=0.0)
    assert XR.shape[1] == D_ESPERADO, "real.npz: D distinto"
    SID, REST, REFR = r_["sesion_id"], r_["est_pose"].astype(np.float64), r_["ref_pose"].astype(np.float64)
    print("\nREAL:", json.loads(str(r_["meta"])).get("aviso"))
    act = slice((K - 1) * NB, K * NB)
    tabla("Desfase de dominio: fotograma actual, sim (train) vs real", pd.DataFrame({
        "feature": NOMBRES, "sim_media": X[m_tr][:, act].mean(0).round(4), "real_media": XR[:, act].mean(0).round(4),
        "sim_desv": X[m_tr][:, act].std(0).round(4), "real_desv": XR[:, act].std(0).round(4)}))
    PR = predecir(XR)
    tabla("Corrección predicha sobre datos reales (|media| y máx, debe ser pequeña)", pd.DataFrame({
        "componente": ["dx [m]", "dy [m]", "dθ [rad]"], "|media|": np.abs(PR).mean(0).round(5), "máx": np.abs(PR).max(0).round(5),
        "|media| en sim (test)": np.abs(Pt_ := predecir(X[m_test])).mean(0).round(5) if m_test.any() else np.nan}))
    filas_r = []
    for sid in np.unique(SID):
        mm = SID == sid
        if mm.sum() < 6: continue
        eb = REST[mm]; incs = np.array([relativa(eb[i - 1], eb[i]) for i in range(1, len(eb))])
        ec = [eb[0].copy()]
        for inc, c in zip(incs, PR[mm][1:]): ec.append(componer(ec[-1], inc + c))
        ec = np.array(ec); largo = lambda P: float(np.hypot(*np.diff(P[:, :2], axis=0).T).sum())
        filas_r.append({"sesión": str(r_["sesiones"][sid]), "fotogramas": int(mm.sum()), "largo_ref_m": round(largo(REFR[mm]), 2),
                        "largo_base_m": round(largo(eb), 2), "largo_corr_m": round(largo(ec), 2)})
    if filas_r:
        dfr = pd.DataFrame(filas_r)
        dfr["|base-ref|"] = (dfr.largo_base_m - dfr.largo_ref_m).abs().round(2); dfr["|corr-ref|"] = (dfr.largo_corr_m - dfr.largo_ref_m).abs().round(2)
        tabla("Consistencia con la pose grabada por el robot (REFERENCIA, no verdad terreno)", dfr)
    fig, axs = plt.subplots(1, 4, figsize=(18, 3.6))
    for ax, nom in zip(axs, ("v_cmd", "gyro_z", "icp_residual", "dmin_front")):
        j = NOMBRES.index(nom)
        ax.hist(X[m_tr][:, (K - 1) * NB + j], bins=50, alpha=.5, density=True, label="sim"); ax.hist(XR[:, (K - 1) * NB + j], bins=50, alpha=.5, density=True, label="real")
        ax.set_title(nom); ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(SALIDA / "07_sim_vs_real.png", dpi=110); plt.show()
else:
    print("\n(sin real.npz: se omite la comparación con datos reales)")

modelo.cpu().eval()
ej = torch.zeros(1, D_ESPERADO)
torch.jit.trace(modelo, ej).save(str(SALIDA / "modelo_correccion.pt"))
try:
    torch.onnx.export(modelo, ej, str(SALIDA / "modelo_correccion.onnx"), input_names=["features"], output_names=["correccion"], opset_version=17)
except Exception as e:
    print("ONNX no exportado (opcional):", e)
W = [p.detach().numpy() for p in modelo.parameters()]            # W1,b1,W2,b2,W3,b3,W4,b4
np.savez(SALIDA / "pesos_mlp.npz", W1=W[0], b1=W[1], W2=W[2], b2=W[3], W3=W[4], b3=W[5], W4=W[6], b4=W[7],
         x_mean=mu_x, x_scale=sd_x, y_mean=mu_y, y_scale=sd_y, K=np.array(K), nombres=np.array(NOMBRES))
def np_forward(x):
    z = (x - mu_x) / sd_x
    for i, (Wm, b) in enumerate(zip(W[0::2], W[1::2])):
        z = z @ Wm.T + b
        if i < 3: z = np.maximum(z, 0)
    return z * sd_y + mu_y
x0 = X[m_test][:64] if m_test.any() else X[:64]
dif = float(np.abs(np_forward(x0) - predecir(x0)).max())
print("diferencia NumPy vs PyTorch (debe ser ~1e-6):", dif)
n_rep = 2000; t0 = time.perf_counter()
for _ in range(n_rep): np_forward(x0[:1])
t_inf_ms = (time.perf_counter() - t0) / n_rep * 1000
print(f"tiempo de inferencia NumPy, 1 muestra, CPU de Kaggle: {t_inf_ms:.3f} ms (en ARM Cortex-A78AE a 1,5 GHz, estimar ~5-10x)")
(SALIDA / "inferencia_ejemplo.py").write_text('''"""Ejemplo: aplicar la corrección en el robot (solo NumPy). Carga pesos_mlp.npz y devuelve la corrección del incremento.
features: vector de K*12 valores (ver nombres en el npz): por fotograma v_cmd, w_cmd, gyro_z, icp_residual, dmin_front, dmin_izq,
dmin_der, prev_dx, prev_dy, prev_dth, scan_ok, dt (los K=5 últimos fotogramas, del más antiguo al más reciente)."""
import numpy as np
p = np.load("pesos_mlp.npz", allow_pickle=False)
def corregir(features):
    z = (np.asarray(features, float) - p["x_mean"]) / p["x_scale"]
    for i, (W, b) in enumerate(((p["W1"], p["b1"]), (p["W2"], p["b2"]), (p["W3"], p["b3"]), (p["W4"], p["b4"]))):
        z = z @ W.T + b
        if i < 3: z = np.maximum(z, 0)
    return z * p["y_scale"] + p["y_mean"]          # (dx, dy, dtheta) a SUMAR al incremento estimado (marco del robot)
if __name__ == "__main__":
    K = int(p["K"]); x = np.zeros(K * 12); x[[4, 5, 6]] = 8.0
    print("corrección de ejemplo (dx, dy, dth):", corregir(x))
''')

def verdict(ok): return "PASA" if ok else "NO PASA"
cj = "test" if any(r["conjunto"] == "test" for r in filas) else "train"
ate_c, rpe_c, rpe_g, cie_c = media(cj, "ATE_corr"), media(cj, "RPE_corr"), media(cj, "RPEgrados_corr"), media(cj, "cierre_corr")
tabla(f"ACEPTACIÓN (conjunto '{cj}': escenarios no vistos)", pd.DataFrame([
    {"criterio": "ATE ≤ 10 cm", "veredicto": verdict(ate_c <= 0.10), "medido": f"{ate_c*100:.1f} cm", "base sin corrección": f"{media(cj,'ATE_base')*100:.1f} cm"},
    {"criterio": "RPE a 1 m ≤ 3 cm y ≤ 2°", "veredicto": verdict(rpe_c <= 0.03 and rpe_g <= 2.0), "medido": f"{rpe_c*100:.1f} cm, {rpe_g:.2f}°",
     "base sin corrección": f"{media(cj,'RPE_base')*100:.1f} cm, {media(cj,'RPEgrados_base'):.2f}°"},
    {"criterio": "Cierre de lazo ≤ 15 cm", "veredicto": "NO EVALUABLE (escenarios de ida)", "medido": f"error final {cie_c*100:.1f} cm ({verdict(cie_c <= 0.15)} como sustituto)",
     "base sin corrección": f"{media(cj,'cierre_base')*100:.1f} cm"},
    {"criterio": "Inferencia < 50 ms", "veredicto": verdict(t_inf_ms * 10 < 50), "medido": f"{t_inf_ms:.3f} ms (CPU Kaggle); ~{t_inf_ms*10:.2f} ms estimados en ARM, NO medidos allí", "base sin corrección": "-"}]))
print("""
LO QUE SE PUDO DEMOSTRAR: el pipeline completo (dataset -> entrenamiento -> exportación -> inferencia NumPy) corre, es reproducible por semilla y se
mide en escenarios NO vistos contra la línea base sin corrección.
LO QUE NO: funcionamiento en el robot real (física del simulador = hipótesis sin calibrar); cierre de lazo (escenarios de ida); tiempo en ARM (solo estimado);
exteriores, LIDAR real con reflejos/vidrio/polvo; con datos reales solo se comparan distribuciones (no hay verdad terreno).
HIPÓTESIS: sesgo de giroscopio 0,2-1 °/s (signo aleatorio) y ruido 0,3 °/s; σ LIDAR 1-3 cm por episodio; odometría base = ICP punto a punto con estimación inicial
(v comandado, giroscopio), no rf2o ni slam_toolbox reales; ventana K=5; se corrige el incremento, no la pose absoluta.
SIGUIENTE PASO: grabar un rosbag real (scan + imu + cmd_vel + pose de referencia) y medir la línea base real. Archivos en""", SALIDA)
