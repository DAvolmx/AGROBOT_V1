# Reentrenar el corrector de SLAM en Kaggle con los datos de hoy (2026-10-23)

**Estado:** el entrenamiento local ya se hizo en la Jetson (misma arquitectura y datos) y **NO mejoró** al ICP crudo en datos reales
(ver `docs/REPORTE_SIMULACION_20261023`). Kaggle sirve para repetirlo con más épocas/GPU o con más sesiones nuevas; no hay credenciales de
Kaggle en la Jetson, así que la subida la haces tú.

1. Datos ya listos en la Jetson: `~/.agrobot/slam/kaggle_hoy/` = `dataset_calibrado.npz` (simulación, 20 MB) + `real_hoy.npz` (7.553 fotogramas reales,
   solo números: sin imágenes) + `pesos_existentes/` (los del 2026-10-21) + `dataset-metadata.json` (id `davidgomez32/agrobot-slam-hoy`, privado).
2. Subir: `kaggle datasets create -p ~/.agrobot/slam/kaggle_hoy` (o *Create dataset* en la web arrastrando esa carpeta).
3. Notebook: *Code → New Notebook → Add input → agrobot-slam-hoy*; pega `agrobot_slam_hoy.py` en una celda, antes de ella:
   ```python
   import os
   os.environ["AGROBOT_SLAM_DIR"] = "/kaggle/input/agrobot-slam-hoy"
   os.environ["AGROBOT_PESOS"] = "/kaggle/input/agrobot-slam-hoy/pesos_existentes"
   os.environ["AGROBOT_SALIDA"] = "/kaggle/working"
   import sys; sys.argv = ["x", "--peso-real", "20", "--retener", "2", "--epocas", "150", "--sim-max", "272000"]
   ```
   Activa GPU. Resultados: `resultados.json` (ATE/RPE por sesión: base vs existente vs sim+real) y `pesos_mlp.npz` + `scaler.pkl`.
4. Solo si `sim+real` gana a `base` en la sesión retenida y en las demás: copia los pesos a `jetson/backend/models/slam_correccion/` y activa
   `slam.correccion_ml_activa` (apagado hoy). Con los datos de hoy NO se cumple.
