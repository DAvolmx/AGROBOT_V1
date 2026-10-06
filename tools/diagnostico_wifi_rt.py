#!/usr/bin/env python3
"""
Diagnóstico en tiempo real de WiFi: monitorea caídas y recuperación.

Uso: python3 diagnostico_wifi_rt.py [intervalo_segundos]
"""
import sys
import json
import time
import subprocess
from pathlib import Path

# Agregar backend al path
sys.path.insert(0, str(Path(__file__).parent.parent / "agrobot_system_2/jetson/backend"))

from diagnostico_wifi import DiagnosticoWiFi


class MonitorWiFiRT:
    def __init__(self):
        self.diag = DiagnosticoWiFi(ventana_s=120.0)
        self.diag.set_callbacks(
            on_patron=self._patron_detectado,
            on_critica=self._caida_critica,
        )
        self.patrones = []

    def _patron_detectado(self, tipo: str, datos: dict):
        """Callback cuando se detecta un patrón."""
        self.patrones.append({
            "tipo": tipo,
            "tiempo": time.time(),
            "datos": datos,
        })
        print(f"\n[!] PATRÓN DETECTADO: {tipo}")
        print(f"    {json.dumps(datos, indent=2)}\n", flush=True)

    def _caida_critica(self, datos: dict):
        """Callback para caídas críticas (ambas vías)."""
        print(f"\n[!!!] CAÍDA CRÍTICA: ambas vías sin conexión")
        print(f"    {json.dumps(datos, indent=2)}\n", flush=True)

    async def leer_estado_sistema(self):
        """Lee estado actual del sistema de conectividad."""
        # Leer info de interfaces
        try:
            resultado = subprocess.run(
                ["ip", "link", "show"],
                capture_output=True, text=True, timeout=5
            )
            interfaces = {}
            for linea in resultado.stdout.split('\n'):
                if ': ' in linea and not linea.startswith(' '):
                    partes = linea.split(':')
                    if len(partes) >= 2:
                        nombre = partes[1].strip()
                        estado = "UP" if "UP" in linea else "DOWN"
                        interfaces[nombre] = estado
        except Exception as e:
            interfaces = {"error": str(e)}

        # Estado de conexiones
        try:
            resultado = subprocess.run(
                ["nmcli", "connection", "show", "--active"],
                capture_output=True, text=True, timeout=5
            )
            conexiones = resultado.stdout.count("ACTIVE")
        except Exception:
            conexiones = "?"

        # Potencia de señal WiFi
        try:
            resultado = subprocess.run(
                ["iw", "dev", "wlan0", "link"],
                capture_output=True, text=True, timeout=5
            )
            rssi = None
            for linea in resultado.stdout.split('\n'):
                if "RSSI:" in linea:
                    # Extraer valor: "RSSI: -45 dBm"
                    partes = linea.split()
                    for i, p in enumerate(partes):
                        if p == "RSSI:" and i + 1 < len(partes):
                            rssi = partes[i + 1]
                            break
            if not rssi:
                rssi = "N/A"
        except Exception:
            rssi = "error"

        return {
            "interfaces": interfaces,
            "conexiones_activas": conexiones,
            "rssi_dbm": rssi,
        }

    def mostrar_dashboard(self):
        """Muestra dashboard de diagnóstico."""
        print("\n" + "=" * 70)
        print("DIAGNÓSTICO DE WiFi - AGROBOT")
        print("=" * 70)

        # Estado BLE/WiFi
        estado = self.diag.resumen()
        print(f"\nESTADO ACTUAL:")
        print(f"  BLE:                {('✓' if estado['ble'] else '✗')}")
        print(f"  WiFi:               {('✓' if estado['wifi'] else '✗')}")
        print(f"  Estado combinado:   {self.diag._describir_estado(estado['ble'], estado['wifi'])}")

        # Estadísticas de caídas
        print(f"\nESTADÍSTICAS:")
        print(f"  Caídas BLE total:   {estado['caidas_ble']}")
        print(f"  Caídas WiFi total:  {estado['caidas_wifi']}")
        print(f"  Caídas simultáneas: {estado['caidas_simultaneas']}")
        print(f"  Recuperaciones:     {len(self.diag.tiempos_recuperacion)}")
        print(f"  Tiempo prom recup:  {estado['tiempo_promedio_recuperacion_s']} s")

        # Análisis
        if self.diag.eventos:
            try:
                import asyncio
                analisis = asyncio.run(self.diag.analizar_caidas())
                print(f"\nANÁLISIS:")
                print(f"  Salud:              {analisis['salud'].upper()}")
                print(f"  Eventos en ventana: {analisis['eventos_en_ventana']}/{analisis['eventos_totales']}")
            except Exception as e:
                print(f"  Error análisis: {e}")

        # Patrones recientes
        if self.patrones:
            print(f"\nPATRONES DETECTADOS (últimos 5):")
            for patron in self.patrones[-5:]:
                print(f"  · {patron['tipo']} a {time.strftime('%H:%M:%S', time.localtime(patron['tiempo']))}")

        print("\n" + "=" * 70)

    def procesar_evento(self, ble: bool, wifi: bool):
        """Procesa un cambio de estado."""
        self.diag.actualizar_estado(ble, wifi)


def main():
    intervalo = int(sys.argv[1]) if len(sys.argv) > 1 else 5

    monitor = MonitorWiFiRT()

    print(f"Iniciando monitoreo (actualización cada {intervalo}s)...")
    print("Presiona Ctrl+C para detener.\n")

    tiempo_inicio = time.time()

    try:
        while True:
            # Simular lecturas (en integración real vendría del servidor)
            # Por ahora usamos valores aleatorios para demo
            import random
            ble_up = random.random() > 0.1  # 90% disponibilidad BLE
            wifi_up = random.random() > 0.15  # 85% disponibilidad WiFi

            monitor.procesar_evento(ble_up, wifi_up)
            monitor.mostrar_dashboard()

            try:
                time.sleep(intervalo)
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass

    # Resumen final
    print("\n" + "=" * 70)
    print("RESUMEN FINAL")
    print("=" * 70)
    tiempo_total = time.time() - tiempo_inicio
    estado = monitor.diag.resumen()

    print(f"Duración monitoreo:    {tiempo_total:.1f} s")
    print(f"Caídas BLE:            {estado['caidas_ble']}")
    print(f"Caídas WiFi:           {estado['caidas_wifi']}")
    print(f"Caídas simultáneas:    {estado['caidas_simultaneas']}")
    print(f"Tasa caída simultánea: {100*estado['caidas_simultaneas']/max(1, estado['caidas_ble']+estado['caidas_wifi']):.1f}%")

    if monitor.diag.tiempos_recuperacion:
        import statistics
        tiempos = list(monitor.diag.tiempos_recuperacion)
        print(f"\nRecuperación:")
        print(f"  Mínimo:              {min(tiempos):.1f} s")
        print(f"  Máximo:              {max(tiempos):.1f} s")
        print(f"  Promedio:            {statistics.mean(tiempos):.1f} s")

    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
