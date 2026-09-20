#!/bin/bash
# Espera a que termine el entrenamiento, valida y monta el mejor modelo
cd /home/jetson/Desktop/AGROBOT/agrobot_system_2

echo "$(date) — esperando a que termine el entrenamiento..."
while pgrep -f "entrenar.py" > /dev/null 2>&1; do
    sleep 60
done
echo "$(date) — entrenamiento terminado"

echo "$(date) — validando modelos..."
python3 -u simulador/historial.py --validar --segundos 90 2>&1 | tee /tmp/validacion.log

echo "$(date) — montando el mejor modelo..."
python3 -c "
import sys
sys.path.insert(0, 'simulador')
import historial, shutil
m = historial.mejor()
if m:
    src = m['ruta']
    dst = 'jetson/backend/politica.pt'
    shutil.copy2(src, dst)
    historial.anotar('despliegue', {'modelo': m['archivo'], 'puntos': m['puntos']})
    print(f'Desplegado: {m[\"archivo\"]} ({m[\"puntos\"]:.1f} puntos)')
    print(historial.tabla())
else:
    print('No hay modelos validados')
"

echo "$(date) — reiniciando backend..."
sudo systemctl restart agrobot-backend
echo "$(date) — listo. El mejor modelo está montado."
