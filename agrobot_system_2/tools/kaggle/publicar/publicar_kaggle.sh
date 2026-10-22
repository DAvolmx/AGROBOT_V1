#!/bin/bash
# Sube dataset.npz como Dataset PRIVADO y publica el notebook (privado, con GPU) en la cuenta de Kaggle del operador.
# Requiere ~/.kaggle/kaggle.json (NO se versiona) y el CLI `kaggle`. Uso: publicar_kaggle.sh <dataset.npz> [binario kaggle] [real.npz opcional]
set -eu
NPZ="$1"; KG="${2:-kaggle}"
AQUI="$(cd "$(dirname "$0")" && pwd)"
cp "$NPZ" "$AQUI/dataset/dataset.npz"; [ -n "${3:-}" ] && [ -f "$3" ] && cp "$3" "$AQUI/dataset/real.npz"
if "$KG" datasets status davidgomez32/agrobot-slam-correction >/dev/null 2>&1; then
  "$KG" datasets version -p "$AQUI/dataset" -m "dataset recalibrado con datos reales del robot" 2>&1
else
  "$KG" datasets create -p "$AQUI/dataset" 2>&1
fi
echo "esperando a que Kaggle procese el dataset…"
for i in $(seq 1 60); do
  s=$("$KG" datasets status davidgomez32/agrobot-slam-correction 2>&1 || true)
  echo "  [$i] $s"; echo "$s" | grep -qi "ready" && break; sleep 20
done
"$KG" kernels push -p "$AQUI/notebook" 2>&1
rm -f "$AQUI/dataset/dataset.npz" "$AQUI/dataset/real.npz"
