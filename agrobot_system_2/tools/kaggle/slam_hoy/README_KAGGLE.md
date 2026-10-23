# Reentrenar el corrector de SLAM en Kaggle con los datos de hoy (2026-10-23)

**Estado (11:30):** YA se entrenó en Kaggle (kernel privado `davidgomez32/agrobot-slam-hoy-entrenar`, GPU, 150 épocas): resultados en `simulador_pasivo/resultados_slam_kaggle_20261023.json`; **empeora** al ICP crudo (ATE 0,37 m vs 0,11 m en la sesión retenida). El entrenamiento local ya se hizo en la Jetson (misma arquitectura y datos) y **NO mejoró** al ICP crudo en datos reales
(ver `docs/REPORTE_SIMULACION_20261023`). Kaggle sirve para repetirlo con más épocas/GPU o con más sesiones nuevas; no hay credenciales de
Kaggle en la Jetson, así que la subida la haces tú.

1. Datos: la simulación calibrada YA está en Kaggle (`davidgomez32/agrobot-slam-correction`, `dataset.npz`, 20 MB). Lo nuevo es un dataset pequeño
   `davidgomez32/agrobot-slam-real-hoy` (7.553 fotogramas reales de hoy, solo números, + los pesos del 21-oct); carpeta lista en `~/.agrobot/slam/kaggle_real/`.
   (Subir los 20 MB desde la Jetson no es viable: el uplink va a ~2 kB/s.) Subir: `kaggle datasets create -p ~/.agrobot/slam/kaggle_real`.
2. Notebook/script: `kernel-metadata.json` + `agrobot_slam_hoy_kaggle.py` en `~/.agrobot/slam/kaggle_kernel/` (privado, GPU, fuentes: los dos datasets):
   `kaggle kernels push -p ~/.agrobot/slam/kaggle_kernel` y `kaggle kernels output davidgomez32/agrobot-slam-hoy-entrenar -p salida`.
3. Resultados: `resultados.json` (ATE/RPE por sesión: base vs existente vs sim+real) y `pesos_mlp.npz` + `scaler.pkl`.
4. Solo si `sim+real` gana a `base` en la sesión retenida y en las demás: copia los pesos a `jetson/backend/models/slam_correccion/` y activa
   `slam.correccion_ml_activa` (apagado hoy). Con los datos de hoy NO se cumple.
