# Video y presentación final de la tesis

`build4.py` genera, desde una sola lista de escenas:
- la presentación PPTX (logo UTA, guion en las notas);
- el guion;
- el video narrado (voz Piper es_MX, subtítulos y clips reales).

Se corre desde `~/video_ldz`, que tiene el entorno `venv` (piper-tts y python-pptx) y las voces en `voz/`. `assets/` tiene los gráficos usados.

```
venv/bin/python build4.py todo              # PPTX con videos + video
LIVIANA=1 venv/bin/python build4.py pptx    # PPTX sin videos (cabe en GitHub)
```

Las salidas finales están en `docs/INFORME_FINAL_TESIS_20261112/`. La presentación con videos (104 MB) no está en git porque supera los 100 MB por archivo de GitHub; se regenera con el comando de arriba.
