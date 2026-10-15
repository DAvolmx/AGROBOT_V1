# Entrenar en Kaggle (no en la Jetson)

1. kaggle.com → New Notebook → File → Import → sube `entrenar_agrobot_kaggle.ipynb`.
2. Settings: Accelerator **GPU T4/P100**, Internet **On**.
3. Repo privado: Add-ons → Secrets → `GH_TOKEN` (token con permiso de lectura del repo).
4. Run All. Al final descarga `resultados.zip` (pesos `.pt` + métricas).
5. En la Jetson, copia el `.pt` de YOLO a `jetson/backend/models/` (para interior:
   `agrobot_interior_obj_yolo11n.pt`) y cárgalo con el botón del dashboard. NO se despliega solo.

El dataset es sintético: mide dominio simulado, no el mundo real. La mosca blanca (paso 3) es un
modelo parcial entrenado con yuca; valida con tus fotos antes de confiar en él.
