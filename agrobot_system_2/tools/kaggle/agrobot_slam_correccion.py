# %% [markdown]
# # AGROBOT · Modelo de corrección de SLAM 2D (odometría por scans)
# Robot agrícola skid-steer SIN encoders · RPLIDAR C1 · MPU6050 · ROS 2 Jazzy en Jetson Orin Nano (ARM64).
#
# **Ciclo cerrado:** el simulador de AGROBOT (`simulador_pasivo/generar_dataset_slam.py`) genera `dataset.npz` → este notebook entrena
# un MLP pequeño → exporta `pesos_mlp.npz` + `scaler.pkl` + TorchScript → el robot los carga con `jetson/backend/slam_correccion.py`
# (solo NumPy).
#
# **Qué aprende el modelo:** la odometría estimada por casamiento de scans (ICP, equivalente a rf2o) se equivoca de forma sistemática
# (p. ej. se queda corta en pasillos largos, donde el scan no restringe el avance). El modelo predice el **error del incremento**
# (real − estimado, en el marco del robot) a partir de una ventana de K=5 fotogramas; al integrar los incrementos corregidos se reduce
# el error de pose. Entrenar con el error de pose *acumulado* directamente no es observable desde una ventana corta, por eso el
# dataset trae `targets` (acumulado, como pide el documento) y `targets_inc` (el que se entrena); la evaluación SIEMPRE se hace sobre
# la pose integrada.

# %%
import json, math, os, pickle, random, time
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SEMILLA = 0
random.seed(SEMILLA); np.random.seed(SEMILLA); torch.manual_seed(SEMILLA); torch.cuda.manual_seed_all(SEMILLA)
torch.backends.cudnn.deterministic = True
DEVICE = "cuda" if (torch.cuda.is_available() and not os.environ.get("AGROBOT_CPU")) else "cpu"
SALIDA = Path(os.environ.get("AGROBOT_SALIDA", "/kaggle/working")); SALIDA.mkdir(parents=True, exist_ok=True)
RUTA = Path(os.environ.get("AGROBOT_NPZ", "/kaggle/input/agrobot-slam-correction/dataset.npz"))
if not RUTA.exists():
    raise FileNotFoundError(f"Falta el dataset: se esperaba {RUTA}. Genera dataset.npz con el simulador "
                            "(generar_dataset_slam.py) y súbelo como Dataset privado de Kaggle con el slug 'agrobot-slam-correction'.")
print("dispositivo:", DEVICE, "· dataset:", RUTA)

# %% [markdown]
# ## 1. Carga y EDA mínimo

# %%
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

# %% [markdown]
# ## 2. Split honesto
# * **Prueba (test):** 2 escenarios NO vistos en entrenamiento.
# * **Entrenamiento:** los demás escenarios; de ellos, las 2 últimas semillas se reservan como *dev* (split temporal por semilla) para el
#   *early stopping*, así los escenarios de prueba no se usan para elegir el checkpoint.

# %%
TODOS = sorted(set(ESC))
VAL_ESC = os.environ.get("AGROBOT_VAL_ESC", "curva_pasillo,hileras_cuadricula").split(",")
faltan = [e for e in VAL_ESC if e not in TODOS]
assert not faltan, f"escenarios de validación ausentes en el dataset: {faltan} (hay {TODOS})"
TRAIN_ESC = [e for e in TODOS if e not in VAL_ESC]
sem_train = sorted(set(SEM[np.isin(ESC, TRAIN_ESC)]))
sem_dev = sem_train[-2:] if len(sem_train) > 2 else sem_train[-1:]
m_test = np.isin(ESC, VAL_ESC)
m_dev = np.isin(ESC, TRAIN_ESC) & np.isin(SEM, sem_dev)
m_tr = np.isin(ESC, TRAIN_ESC) & ~np.isin(SEM, sem_dev)
print("train:", TRAIN_ESC, "semillas", [s for s in sem_train if s not in sem_dev])
print("dev  :", TRAIN_ESC, "semillas", sem_dev)
print("test :", VAL_ESC, "(NO vistos)")
print("muestras train/dev/test:", int(m_tr.sum()), int(m_dev.sum()), int(m_test.sum()))
assert m_tr.sum() > 0 and m_dev.sum() > 0 and m_test.sum() > 0, "split vacío: el dataset necesita >=3 semillas y escenarios de train y de test"

# %% [markdown]
# ## 3. Preprocesado (normalización ajustada SOLO con train)

# %%
mu_x, sd_x = X[m_tr].mean(0), X[m_tr].std(0) + 1e-6
mu_y, sd_y = Y_INC[m_tr].mean(0), Y_INC[m_tr].std(0) + 1e-9
nx = lambda A: ((A - mu_x) / sd_x).astype(np.float32)
ny = lambda A: ((A - mu_y) / sd_y).astype(np.float32)
with open(SALIDA / "scaler.pkl", "wb") as f:      # dict simple (sin sklearn) para que el robot lo cargue con NumPy
    pickle.dump({"mean": mu_x, "scale": sd_x, "y_mean": mu_y, "y_scale": sd_y, "K": K, "nombres": NOMBRES}, f)
tt = lambda a: torch.tensor(a, device=DEVICE)
Xtr, Ytr, Xdv, Ydv = tt(nx(X[m_tr])), tt(ny(Y_INC[m_tr])), tt(nx(X[m_dev])), tt(ny(Y_INC[m_dev]))

# %% [markdown]
# ## 4. Modelo: MLP 128-64-32 (opción A)
# Se elige el MLP sobre la LSTM porque la ventana de K=5 ya viene aplanada, tiene ~19k parámetros (< 100k) y se evalúa con tres
# multiplicaciones de matrices: sub-milisegundo en un núcleo ARM, y se puede ejecutar en NumPy puro en el robot.

# %%
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

# %% [markdown]
# ## 5. Entrenamiento (Huber, Adam 1e-3, batch 64, paciencia 15, máx. 150 épocas)

# %%
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

# %% [markdown]
# ## 6. Evaluación: pose integrada con los incrementos corregidos vs. la línea base SIN corrección
# * **ATE:** RMSE de posición tras alineación rígida 2D (Umeyama sin escala).
# * **RPE a 1 m:** error de la pose relativa entre puntos separados 1 m de recorrido real (traslación en m, rotación en °).
# * **Cierre de lazo:** los escenarios del simulador son de ida (no vuelven al inicio): se reporta el **error final de posición** como
#   sustituto, marcado así. Hay que añadir episodios de ida y vuelta al simulador para medir el cierre real.

# %%
def componer(p, inc):
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([p[0] + c * inc[0] - s * inc[1], p[1] + s * inc[0] + c * inc[1], (p[2] + inc[2] + math.pi) % (2 * math.pi) - math.pi])

def relativa(pj, pk):
    c, s = math.cos(pj[2]), math.sin(pj[2]); dx, dy = pk[0] - pj[0], pk[1] - pj[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, (pk[2] - pj[2] + math.pi) % (2 * math.pi) - math.pi])

def predecir(Xe):
    with torch.no_grad():
        p = modelo(tt(nx(Xe))).cpu().numpy()
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
for conj in ("train", "test"):
    if not any(r["conjunto"] == conj for r in filas): continue
    print(f"--- {conj} ({sum(r['conjunto'] == conj for r in filas)} episodios) ---")
    for k in ("ATE", "RPE", "RPEgrados", "cierre"):
        b, c = media(conj, k + "_base"), media(conj, k + "_corr")
        print(f"{k:10s} base {b:8.4f} · corregido {c:8.4f} · mejora {100 * (b - c) / max(b, 1e-9):6.1f} %")
print("\nPor falla (test):")
for fl in sorted(set(r["falla"] for r in filas if r["conjunto"] == "test")):
    rs = [r for r in filas if r["conjunto"] == "test" and r["falla"] == fl]
    print(f"  {fl:18s} ATE {np.mean([r['ATE_base'] for r in rs]):.3f} -> {np.mean([r['ATE_corr'] for r in rs]):.3f}")

# %% [markdown]
# ## 7. Gráficos

# %%
def dibujar(ep, titulo, ax):
    real, b, c = trayectorias[ep]
    ax.plot(real[:, 0], real[:, 1], "k-", lw=2, label="real"); ax.plot(b[:, 0], b[:, 1], "r--", label="estimada (base)")
    ax.plot(c[:, 0], c[:, 1], "g-", label="corregida"); ax.set_aspect("equal"); ax.set_title(titulo, fontsize=9); ax.legend(fontsize=7); ax.grid(alpha=.3)
fig, axs = plt.subplots(1, 3, figsize=(16, 4.5))
epi = {c: [e for e in trayectorias if (ESC[EP == e][0] in VAL_ESC) == (c == "test")] for c in ("train", "test")}
for ax, c in zip(axs[:2], ("train", "test")):
    if epi[c]:
        e = epi[c][0]; dibujar(e, f"{c}: {ESC[EP == e][0]} · {FALLA[EP == e][0]}", ax)
axs[2].plot([h[0] for h in hist], label="train"); axs[2].plot([h[1] for h in hist], label="dev"); axs[2].set_title("pérdida (Huber)"); axs[2].legend(); axs[2].grid(alpha=.3)
fig.tight_layout(); fig.savefig(SALIDA / "trayectorias_y_perdida.png", dpi=120); plt.close(fig)
print("guardado", SALIDA / "trayectorias_y_perdida.png")

# %% [markdown]
# ## 8. Exportación para el robot

# %%
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

# %% [markdown]
# ## 9. Resumen de aceptación

# %%
def verdict(ok): return "PASA" if ok else "NO PASA"
cj = "test" if any(r["conjunto"] == "test" for r in filas) else "train"
ate_c, rpe_c, rpe_g, cie_c = media(cj, "ATE_corr"), media(cj, "RPE_corr"), media(cj, "RPEgrados_corr"), media(cj, "cierre_corr")
print(f"Criterios medidos sobre el conjunto '{cj}' (escenarios no vistos):")
print(f"  ATE ≤ 10 cm             : {verdict(ate_c <= 0.10):8s} ({ate_c * 100:.1f} cm; base {media(cj, 'ATE_base') * 100:.1f} cm)")
print(f"  RPE a 1 m ≤ 3 cm y ≤ 2° : {verdict(rpe_c <= 0.03 and rpe_g <= 2.0):8s} ({rpe_c * 100:.1f} cm, {rpe_g:.2f}°; base {media(cj, 'RPE_base') * 100:.1f} cm, {media(cj, 'RPEgrados_base'):.2f}°)")
print(f"  Cierre de lazo ≤ 15 cm  : NO EVALUABLE (escenarios de ida); error FINAL medio {cie_c * 100:.1f} cm ({verdict(cie_c <= 0.15)} como sustituto)")
print(f"  Inferencia < 50 ms      : {verdict(t_inf_ms * 10 < 50):8s} ({t_inf_ms:.3f} ms medidos; ~{t_inf_ms * 10:.2f} ms estimados en ARM, NO medidos allí)")

# %% [markdown]
# ## 10. Lo que se pudo demostrar y lo que NO
# **Demostrado (en simulación):** el pipeline completo (dataset → entrenamiento → exportación → inferencia en NumPy) corre y es
# reproducible por semilla; las métricas se miden sobre escenarios **no vistos** y se comparan con la línea base sin corrección.
#
# **NO demostrado:**
# * Que funcione en el robot real: la física del simulador es una **hipótesis sin calibrar** (`calibrated=false`), ajustada a pocas
#   medidas reales (giro ≈ 10–15° y ≈ 10 cm de avance por toque; el simulador avanza menos al girar).
# * El **cierre de lazo** (los escenarios no vuelven al inicio) y la inferencia **en ARM** (solo se estima).
# * Robustez frente a entornos exteriores o a LIDAR reales con reflejos, vidrio real, polvo.
#
# **Hipótesis usadas:** sesgo del giroscopio 0,2–1 °/s con signo aleatorio y ruido blanco 0,3 °/s; σ del LIDAR 1–3 cm (sorteada por
# episodio); odometría base = ICP punto a punto con estimación inicial (v comandado, giroscopio), no rf2o ni slam_toolbox reales; ventana K=5;
# el modelo corrige el incremento, no la pose absoluta; los targets de validación son 2 escenarios del mismo simulador (generalización a
# escenarios nuevos del MISMO mundo simulado, no a un mundo real).
#
# **Siguiente paso honesto:** grabar un rosbag real (scan + imu + cmd_vel + pose de referencia) y volver a medir la línea base real
# antes de confiar en la corrección.
