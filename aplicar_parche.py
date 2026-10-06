#!/usr/bin/env python3
"""
Script automático que aplica el parche de comunicación a jetson_server.py

Uso:
    python3 aplicar_parche.py

    o si está en otra ruta:
    python3 aplicar_parche.py --servidor /ruta/a/jetson_server.py
"""
import sys
import argparse
from pathlib import Path
import shutil


def aplicar_parche(ruta_servidor):
    """Aplica todas las modificaciones necesarias al servidor."""

    ruta = Path(ruta_servidor)

    if not ruta.exists():
        print(f"✗ Archivo no encontrado: {ruta}")
        return False

    print(f"📝 Aplicando parche a: {ruta}")

    # Backup
    backup = ruta.with_suffix('.py.backup')
    shutil.copy2(ruta, backup)
    print(f"✓ Backup guardado: {backup}")

    # Leer contenido
    contenido = ruta.read_text()

    # 1. AGREGAR IMPORT
    if "from auto_comunicacion import inicializar_auto_comunicacion" not in contenido:
        # Buscar línea de imports (después de "from politica import Politica")
        if "from politica import Politica" in contenido:
            contenido = contenido.replace(
                "from politica import Politica",
                "from politica import Politica\nfrom auto_comunicacion import inicializar_auto_comunicacion"
            )
            print("✓ Import agregado")
        else:
            print("⚠ No se pudo encontrar dónde agregar el import")

    # 2. AGREGAR ATRIBUTO EN __init__
    if "self.auto_comm = None" not in contenido:
        # Buscar dónde agregar (después de self.enlace_c3)
        if "self.enlace_c3" in contenido:
            lineas = contenido.split('\n')
            for i, linea in enumerate(lineas):
                if "self.enlace_c3 = None" in linea:
                    lineas.insert(i + 1, "        self.auto_comm = None")
                    contenido = '\n'.join(lineas)
                    print("✓ Atributo auto_comm agregado a __init__")
                    break

    # 3. AGREGAR INICIALIZACIÓN EN start()
    if "await inicializar_auto_comunicacion" not in contenido:
        # Buscar dónde agregar (después de inicializar enlace_c3)
        if "self.enlace_c3 = EnlaceC3" in contenido:
            parche_start = '''
        # --- COMUNICACIÓN AUTOMÁTICA ---
        try:
            self.auto_comm = await inicializar_auto_comunicacion(
                config_store=self.config_store,
                modo_wifi="patrullaje"
            )
            print("[servidor] ✓ Sistema automático iniciado", flush=True)
        except Exception as e:
            print(f"[servidor] ✗ Error auto-comunicación: {e}", flush=True)
            self.auto_comm = None
        # --- FIN COMUNICACIÓN AUTOMÁTICA ---
'''
            # Encontrar el lugar correcto (después de inicializar enlace_c3)
            idx = contenido.find("self.enlace_c3 = EnlaceC3")
            if idx != -1:
                # Encontrar el fin de esa línea
                fin_linea = contenido.find('\n', idx)
                if fin_linea != -1:
                    contenido = contenido[:fin_linea+1] + parche_start + contenido[fin_linea+1:]
                    print("✓ Inicialización agregada en start()")

    # 4. AGREGAR ACTUALIZACIÓN EN BUCLE
    if "self.auto_comm.actualizar_estado_vias" not in contenido:
        # Esto es más complejo, agregar cerca del bucle principal
        parche_bucle = '''            # --- AUTO-COMUNICACIÓN ---
            if self.auto_comm and self.enlace_c3:
                self.auto_comm.actualizar_estado_vias(
                    ble_conectado=self.enlace_c3.ble.connected,
                    wifi_conectado=(self.enlace_c3.wifi.conectado
                                   if self.enlace_c3.wifi else False)
                )
            # --- FIN AUTO-COMUNICACIÓN ---
'''
        # Buscar bucle de control
        if "while not self._stop.is_set():" in contenido:
            idx = contenido.find("while not self._stop.is_set():")
            if idx != -1:
                # Insertar después del while
                fin_while = contenido.find('\n', idx)
                if fin_while != -1:
                    # Encontrar la siguiente línea con código (no comentarios)
                    sig_linea = fin_while + 1
                    contenido = contenido[:sig_linea] + parche_bucle + contenido[sig_linea:]
                    print("✓ Actualización agregada en bucle")

    # 5. AGREGAR CLEANUP EN stop()
    if "await self.auto_comm.detener()" not in contenido:
        if "async def stop(self):" in contenido:
            idx = contenido.find("async def stop(self):")
            if idx != -1:
                # Encontrar el último return de esa función
                resto = contenido[idx:]
                idx_return = resto.rfind("return")
                if idx_return != -1:
                    pos_antes_return = idx + idx_return
                    parche_stop = '''        if self.auto_comm:
            await self.auto_comm.detener()
        '''
                    contenido = contenido[:pos_antes_return] + parche_stop + contenido[pos_antes_return:]
                    print("✓ Cleanup agregado en stop()")

    # Guardar
    ruta.write_text(contenido)
    print(f"\n✅ Parche aplicado exitosamente")
    print(f"📍 Original: {backup}")
    print(f"📍 Modificado: {ruta}")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Aplica parche de comunicación automática a jetson_server.py"
    )
    parser.add_argument(
        "--servidor",
        default="/home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/backend/jetson_server.py",
        help="Ruta al archivo jetson_server.py"
    )

    args = parser.parse_args()

    if aplicar_parche(args.servidor):
        print("\n" + "="*60)
        print("PRÓXIMO PASO: Reinicia el servidor")
        print("="*60)
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
