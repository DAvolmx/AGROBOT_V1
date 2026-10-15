#!/usr/bin/env python3
"""
Test Script para Firmware v3 — AGROBOT C3
Envía comandos de prueba y verifica respuestas
"""

import socket
import struct
import time
import sys

# Configuración
C3_IP = "192.168.0.4"        # IP del C3 (ajusta si es diferente)
C3_UDP_PUERTO = 5002
JETSON_IP = "192.168.0.1"    # IP de la Jetson (AP)

# Protocolo (igual que firmware)
CMD_MAGIC = 0xA5
STATUS_MAGIC = 0x5A

def crc8(data):
    """Calcula CRC-8 del payload"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

class CmdFrame(struct.Struct):
    """Estructura de comando (8 bytes)"""
    fmt = '<BBBBBBHB'
    size = struct.calcsize(fmt)

    @staticmethod
    def create(v_pct, omega_pct, servo_angle=98, flags=0, seq=1):
        data = struct.pack('<BBBBBBH',
            CMD_MAGIC,
            v_pct & 0xFF,
            omega_pct & 0xFF,
            servo_angle & 0xFF,
            flags & 0xFF,
            0,  # reserved
            seq & 0xFFFF)
        crc = crc8(data)
        return data + struct.pack('B', crc)

def enviar_comando(sock, v_pct, omega_pct, servo_angle=98, flags=0, seq=1, verbose=True):
    """Envía comando al C3"""
    frame = CmdFrame.create(v_pct, omega_pct, servo_angle, flags, seq)
    sock.sendto(frame, (C3_IP, C3_UDP_PUERTO))
    if verbose:
        print(f"  📤 Enviado: v={v_pct:3d}% ω={omega_pct:3d}% servo={servo_angle}°")
    return seq + 1

def test_adelante_atras(sock):
    """Prueba: movimiento adelante/atrás"""
    print("\n🔵 TEST 1: Adelante/Atrás")
    print("  Esperado: L1+L2 y R1+R2 avanzan juntas, sin cruce")

    seq = 1
    print("  → Adelante 50%...")
    seq = enviar_comando(sock, 50, 0, seq=seq)
    time.sleep(2)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)

    print("  → Atrás 50%...")
    seq = enviar_comando(sock, -50, 0, seq=seq)
    time.sleep(2)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)
    print("  ✅ Completado")

def test_giro_puro(sock):
    """Prueba: giro puro sin avance"""
    print("\n🟢 TEST 2: Giro Puro (IZQ/DER)")
    print("  Esperado: Lado IZQ y DER independientes, sin cruce en X")

    seq = 1
    print("  → Giro IZQUIERDA (L atrás, D adelante)...")
    seq = enviar_comando(sock, 0, 50, seq=seq)
    time.sleep(3)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)

    print("  → Giro DERECHA (L adelante, D atrás)...")
    seq = enviar_comando(sock, 0, -50, seq=seq)
    time.sleep(3)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)
    print("  ✅ Completado")

def test_giro_encendido(sock):
    """Prueba: verificar que el TEST_MOTORES se ejecutó al arrancar"""
    print("\n🟡 TEST 0: Verifica Serial (antes de este script)")
    print("  Abre Serial Monitor en Arduino IDE o:")
    print("  $ picocom /dev/ttyACM0 -b 115200")
    print()
    print("  Deberías ver:")
    print("    === AGROBOT C3 v3 — SKID-STEER + PWM PURO ===")
    print("    [TEST] R1 adelante...")
    print("    [TEST] R2 adelante...")
    print("    [TEST] L1 adelante...")
    print("    [TEST] L2 adelante...")
    print("    [TEST] Todas adelante 500ms...")
    print("    [TEST] OK")
    print()
    input("  📌 Presiona Enter cuando hayas visto el test en Serial...")
    print("  ✅ Continuando con pruebas de control")

def test_arco_suave(sock):
    """Prueba: movimiento en arco (adelante + giro)"""
    print("\n🔴 TEST 3: Movimiento en Arco")
    print("  Esperado: L lento, D rápido (arco suave)")

    seq = 1
    print("  → Arco ADELANTE-IZQUIERDA...")
    seq = enviar_comando(sock, 50, 30, seq=seq)  # v=50%, omega=30%
    time.sleep(3)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)

    print("  → Arco ADELANTE-DERECHA...")
    seq = enviar_comando(sock, 50, -30, seq=seq)  # v=50%, omega=-30%
    time.sleep(3)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)
    print("  ✅ Completado")

def test_velocidades_max(sock):
    """Prueba: máximas velocidades"""
    print("\n⚫ TEST 4: Máximas Velocidades (100%)")
    print("  Esperado: Torque MÁXIMO, respuesta instantánea")

    seq = 1
    print("  → Adelante MÁXIMO...")
    seq = enviar_comando(sock, 100, 0, seq=seq)
    time.sleep(3)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)

    print("  → Giro MÁXIMO...")
    seq = enviar_comando(sock, 0, 100, seq=seq)
    time.sleep(3)

    print("  → Parar...")
    seq = enviar_comando(sock, 0, 0, seq=seq)
    time.sleep(1)
    print("  ✅ Completado")

def main():
    print("=" * 60)
    print("AGROBOT C3 v3 — Test Suite")
    print("=" * 60)
    print()
    print(f"📡 Conectando a {C3_IP}:{C3_UDP_PUERTO}...")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", 5002))  # Escucha respuestas en 5002
        sock.settimeout(2)
        print(f"✅ Socket abierto")
    except Exception as e:
        print(f"❌ Error al abrir socket: {e}")
        sys.exit(1)

    try:
        # Test 0: Verificar Serial
        test_giro_encendido(sock)

        # Espera a que el usuario esté listo
        time.sleep(1)

        # Test 1: Adelante/Atrás
        test_adelante_atras(sock)

        # Test 2: Giro Puro
        test_giro_puro(sock)

        # Test 3: Arco
        test_arco_suave(sock)

        # Test 4: Máximas velocidades
        test_velocidades_max(sock)

        # Parar al final
        sock.sendto(CmdFrame.create(0, 0, seq=99), (C3_IP, C3_UDP_PUERTO))

        print("\n" + "=" * 60)
        print("✅ TODOS LOS TESTS COMPLETADOS")
        print("=" * 60)
        print()
        print("📝 Observaciones esperadas:")
        print("  • Giro SIN cruce (no forma X)")
        print("  • Lados L y D siempre coordinados")
        print("  • Respuesta INSTANTÁNEA (sin delays)")
        print("  • Máximo torque en arranque")
        print()
        print("❌ Si algo falla:")
        print("  1. Verifica Serial Monitor (velocidades reales)")
        print("  2. Revisa cables de motor y pines")
        print("  3. Comprueba inversiones INVERT_* en código")
        print()

    except KeyboardInterrupt:
        print("\n⚠️ Interrumpido por usuario")
        sock.sendto(CmdFrame.create(0, 0, seq=99), (C3_IP, C3_UDP_PUERTO))
    except Exception as e:
        print(f"❌ Error durante prueba: {e}")
        sock.sendto(CmdFrame.create(0, 0, seq=99), (C3_IP, C3_UDP_PUERTO))
    finally:
        sock.close()
        print("🔌 Socket cerrado")

if __name__ == "__main__":
    main()
