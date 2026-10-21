# Ciclo cerrado: simulador → Kaggle → robot (corrección de SLAM)

```
simulador_pasivo/generar_dataset_slam.py  ──►  dataset.npz  ──►  Kaggle (GPU T4 gratis)
        (mismo simulador del manejo)                              tools/kaggle/agrobot_slam_correccion.ipynb
                                                                          │
robot: jetson/backend/slam_correccion.py  ◄──  pesos_mlp.npz + scaler.pkl ◄┘   (NumPy puro, sin torch)
```

## 1. Generar el dataset (en la Jetson o en cualquier máquina con el repo)
```bash
cd agrobot_system_2/simulador_pasivo
python3 generar_dataset_slam.py --config optimo --procs 6 --semillas 6      # ~1,5 h con 6 núcleos libres; --rapido para probar
```
Genera `~/.agrobot/slam/dataset_<fecha>.npz`. Cada episodio es un `episodio_fallas` (geometría del croquis, firmware, física con fallas,
choques, paradas de escaneo) con el scan, los comandos y la pose verdadera; la odometría «estimada» es ICP con estimación inicial
(v comandado + giroscopio) y se integra: deriva como la del robot (se queda corta en pasillos largos). Ruido LIDAR σ 1–3 cm y sesgo
de IMU 0,2–1 °/s sorteados por episodio (hipótesis). 8 escenarios × 8 fallas × semillas.

## 2. Entrenar en Kaggle
1. Kaggle → *Datasets* → *New Dataset* privado, slug **`agrobot-slam-correction`**, sube el `.npz` renombrado a **`dataset.npz`**.
2. *Code* → *New Notebook* → *File → Import Notebook* → `tools/kaggle/agrobot_slam_correccion.ipynb`; *Add input* → tu dataset;
   acelerador GPU T4 (también corre en CPU). *Run all*.
3. Descarga de `/kaggle/working`: `pesos_mlp.npz`, `scaler.pkl`, `modelo_correccion.pt`, `metricas.csv`, `trayectorias_y_perdida.png`.

El notebook entrena con **2 escenarios sin ver** (`curva_pasillo`, `hileras_cuadricula`), reporta ATE/RPE/cierre contra la línea base sin
corrección y termina con PASA / NO PASA por criterio. Lo que no pueda demostrar lo dice (cierre de lazo no evaluable: los escenarios son de ida).

## 3. Subir al robot (decisión del operador; nada se despliega solo)
Copiar `pesos_mlp.npz` a `jetson/backend/models/` y usar `slam_correccion.CorreccionSlam` en el nodo de odometría:
```python
from slam_correccion import CorreccionSlam
c = CorreccionSlam("models/pesos_mlp.npz")
inc = c.corregir_incremento(inc_icp, v_cmd=v, w_cmd=w, gyro_z=gz, icp_residual=res, rangos=scan360, scan_ok=ok, dt=dt)
```
Seguridades: tope de ±5 cm / ±0,05 rad por paso, ceros si la entrada es no finita o está fuera de distribución (|z| > 8).
Antes de confiar en él en el campo: grabar un rosbag real (scan + imu + cmd_vel + referencia) y medir la línea base real.

## 4. Límites conocidos
- La física es hipótesis sin calibrar; la odometría base es ICP propio, no rf2o ni slam_toolbox reales.
- No hay cierre de lazo medido ni tiempo de inferencia medido en ARM (la prueba unitaria da < 5 ms en esta Jetson).
- El modelo corrige el **incremento** (observable en una ventana de 5 fotogramas), no la pose absoluta.
