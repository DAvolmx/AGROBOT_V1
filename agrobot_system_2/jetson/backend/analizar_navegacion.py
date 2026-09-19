"""
Análisis Automatizado de Datos de Navegación
==============================================

Lee sesiones guardadas y sugiere ajustes de parámetros basados en datos reales.

Uso:
  $ python3 analizar_navegacion.py --session 20260919_143000
"""

import csv
import json
import argparse
from pathlib import Path
from collections import Counter, defaultdict
from statistics import mean, stdev


def leer_poses(poses_file: Path) -> list:
    """Lee archivo CSV de poses."""
    poses = []
    try:
        with poses_file.open('r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    poses.append({
                        'timestamp': float(row['timestamp']),
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'theta_deg': float(row['theta_deg']),
                        'vx_desired': float(row.get('vx_desired', 0)),
                        'omega_desired': float(row.get('omega_desired', 0)),
                        'vx_real': float(row.get('vx_real', 0)),
                        'omega_real': float(row.get('omega_real', 0)),
                        'lidar_frontal': float(row.get('lidar_frontal_cm', 0)) or None,
                        'lidar_izq': float(row.get('lidar_lateral_izq_cm', 0)) or None,
                        'lidar_der': float(row.get('lidar_lateral_der_cm', 0)) or None,
                    })
                except:
                    pass
    except FileNotFoundError:
        pass
    return poses


def leer_events(events_file: Path) -> list:
    """Lee archivo JSONL de eventos."""
    events = []
    try:
        with events_file.open('r') as f:
            for line in f:
                try:
                    events.append(json.loads(line))
                except:
                    pass
    except FileNotFoundError:
        pass
    return events


def leer_obstaculos(obs_file: Path) -> list:
    """Lee archivo JSONL de obstáculos."""
    obstaculos = []
    try:
        with obs_file.open('r') as f:
            for line in f:
                try:
                    obstaculos.append(json.loads(line))
                except:
                    pass
    except FileNotFoundError:
        pass
    return obstaculos


class AnalizadorNavegacion:
    """Analiza datos de navegación y sugiere mejoras."""

    def __init__(self, session_id: str, output_dir: Path = None):
        self.session_id = session_id
        self.output_dir = output_dir or (Path.home() / ".agrobot" / "navigation_logs")

        # Cargar datos
        self.poses = leer_poses(self.output_dir / f"poses_{session_id}.csv")
        self.events = leer_events(self.output_dir / f"events_{session_id}.jsonl")
        self.obstaculos = leer_obstaculos(self.output_dir / f"obstacles_{session_id}.jsonl")

    def analizar_distancias_lidar(self) -> dict:
        """Analiza distribución de distancias LIDAR."""
        if not self.poses:
            return {'error': 'sin datos'}

        frontales = [p['lidar_frontal'] for p in self.poses if p['lidar_frontal'] and p['lidar_frontal'] < 300]
        izquierdas = [p['lidar_izq'] for p in self.poses if p['lidar_izq'] and p['lidar_izq'] < 300]
        derechas = [p['lidar_der'] for p in self.poses if p['lidar_der'] and p['lidar_der'] < 300]

        return {
            'frontal_min_cm': min(frontales) if frontales else None,
            'frontal_promedio_cm': mean(frontales) if frontales else None,
            'frontal_desviacion_cm': stdev(frontales) if len(frontales) > 1 else None,
            'izquierda_promedio_cm': mean(izquierdas) if izquierdas else None,
            'derecha_promedio_cm': mean(derechas) if derechas else None,
            'sector_mas_peligroso': self._sector_critico(frontales, izquierdas, derechas),
        }

    def analizar_velocidades(self) -> dict:
        """Analiza desviación entre velocidades deseadas y reales."""
        if not self.poses:
            return {'error': 'sin datos'}

        # Velocidades lineales
        vx_deseadas = [p['vx_desired'] for p in self.poses if p['vx_desired']]
        vx_reales = [p['vx_real'] for p in self.poses if p['vx_real']]

        # Errores
        errores_vx = []
        for p in self.poses:
            if p['vx_desired'] and p['vx_real']:
                errores_vx.append(abs(p['vx_desired'] - p['vx_real']))

        return {
            'vx_deseada_promedio_ms': mean(vx_deseadas) if vx_deseadas else None,
            'vx_real_promedio_ms': mean(vx_reales) if vx_reales else None,
            'error_velocidad_promedio_ms': mean(errores_vx) if errores_vx else None,
            'error_max_ms': max(errores_vx) if errores_vx else None,
        }

    def analizar_evasiones(self) -> dict:
        """Analiza efectividad de evasiones."""
        if not self.events:
            return {'total': 0}

        evasiones = [e for e in self.events if e.get('tipo') == 'evasion']
        motivos = Counter(e.get('motivo', 'unknown') for e in evasiones)

        return {
            'total': len(evasiones),
            'motivos': dict(motivos),
            'estado_anterior': Counter(e.get('estado_anterior') for e in evasiones),
            'estado_nuevo': Counter(e.get('estado_nuevo') for e in evasiones),
        }

    def analizar_patrulla(self) -> dict:
        """Analiza eventos de patrulla."""
        if not self.events:
            return {'total': 0}

        patrullas = [e for e in self.events if e.get('tipo') == 'patrulla']

        waypoints_alcanzados = len([p for p in patrullas if p.get('evento') == 'llegado'])
        waypoints_timeout = len([p for p in patrullas if p.get('evento') == 'timeout'])
        escaneos = len([p for p in patrullas if p.get('evento') == 'escaneo'])

        return {
            'total_eventos': len(patrullas),
            'waypoints_alcanzados': waypoints_alcanzados,
            'waypoints_timeout': waypoints_timeout,
            'escaneos_completados': escaneos,
        }

    def sugerir_mejoras(self) -> list:
        """Sugiere mejoras basadas en los datos."""
        mejoras = []

        # Análisis de LIDAR
        lidar_stats = self.analizar_distancias_lidar()
        if lidar_stats.get('frontal_min_cm'):
            if lidar_stats['frontal_min_cm'] < 15:
                mejoras.append({
                    'severidad': 'ALTA',
                    'area': 'SEGURIDAD',
                    'problema': f"Frontal llega a {lidar_stats['frontal_min_cm']:.0f}cm (crítico < 15cm)",
                    'recomendacion': 'Bajar DIST_CRITICA_PARADA a 10cm o aumentar velocidad máxima',
                })

        # Análisis de velocidades
        vel_stats = self.analizar_velocidades()
        if vel_stats.get('error_velocidad_promedio_ms'):
            if vel_stats['error_velocidad_promedio_ms'] > 0.1:
                mejoras.append({
                    'severidad': 'MEDIA',
                    'area': 'CONTROL',
                    'problema': f"Error de velocidad: {vel_stats['error_velocidad_promedio_ms']:.3f} m/s promedio",
                    'recomendacion': 'Ajustar PID de control lineal (Kp, Ki, Kd)',
                })

        # Análisis de evasiones
        eva_stats = self.analizar_evasiones()
        if eva_stats['total'] > 10:
            mejoras.append({
                'severidad': 'MEDIA',
                'area': 'EVASION',
                'problema': f"Muchas evasiones ({eva_stats['total']}), posible error en mapeo",
                'recomendacion': 'Verificar calibración de LIDAR y parámetros de inflado de costmap',
            })

        # Análisis de patrulla
        pat_stats = self.analizar_patrulla()
        if pat_stats['waypoints_timeout'] > pat_stats['waypoints_alcanzados']:
            mejoras.append({
                'severidad': 'ALTA',
                'area': 'PATRULLA',
                'problema': f"Más timeouts ({pat_stats['waypoints_timeout']}) que llegadas ({pat_stats['waypoints_alcanzados']})",
                'recomendacion': 'Aumentar GOAL_TIMEOUT_S de 90s o revisar configuración de Nav2',
            })

        return sorted(mejoras, key=lambda x: {'ALTA': 0, 'MEDIA': 1, 'BAJA': 2}[x['severidad']])

    @staticmethod
    def _sector_critico(frontal, izq, der):
        """Identifica qué sector tiene más distancias cortas."""
        if not any([frontal, izq, der]):
            return None
        valores = {
            'frontal': min(frontal) if frontal else 300,
            'izquierda': min(izq) if izq else 300,
            'derecha': min(der) if der else 300,
        }
        return min(valores, key=valores.get)

    def generar_reporte(self) -> str:
        """Genera reporte completo en texto."""
        reporte = []
        reporte.append("=" * 70)
        reporte.append(f"ANÁLISIS DE NAVEGACIÓN — Sesión {self.session_id}")
        reporte.append("=" * 70)
        reporte.append("")

        # Resumen general
        reporte.append(f"Poses registradas: {len(self.poses)}")
        reporte.append(f"Eventos: {len(self.events)}")
        reporte.append(f"Obstáculos detectados: {len(self.obstaculos)}")
        reporte.append("")

        # LIDAR
        reporte.append("─" * 70)
        reporte.append("ANÁLISIS DE LIDAR")
        reporte.append("─" * 70)
        lidar = self.analizar_distancias_lidar()
        for k, v in lidar.items():
            if isinstance(v, float):
                reporte.append(f"  {k:.<40} {v:.1f} cm")
            else:
                reporte.append(f"  {k:.<40} {v}")
        reporte.append("")

        # Velocidades
        reporte.append("─" * 70)
        reporte.append("ANÁLISIS DE CONTROL DE VELOCIDAD")
        reporte.append("─" * 70)
        vel = self.analizar_velocidades()
        for k, v in vel.items():
            if isinstance(v, float):
                reporte.append(f"  {k:.<40} {v:.3f} m/s")
            else:
                reporte.append(f"  {k:.<40} {v}")
        reporte.append("")

        # Evasiones
        reporte.append("─" * 70)
        reporte.append("ANÁLISIS DE EVASIONES")
        reporte.append("─" * 70)
        eva = self.analizar_evasiones()
        reporte.append(f"  {'Total de evasiones':.<40} {eva['total']}")
        if eva.get('motivos'):
            for motivo, count in eva['motivos'].items():
                reporte.append(f"    • {motivo}: {count}")
        reporte.append("")

        # Patrulla
        reporte.append("─" * 70)
        reporte.append("ANÁLISIS DE PATRULLA")
        reporte.append("─" * 70)
        pat = self.analizar_patrulla()
        reporte.append(f"  {'Eventos totales':.<40} {pat['total_eventos']}")
        reporte.append(f"  {'Waypoints alcanzados':.<40} {pat['waypoints_alcanzados']}")
        reporte.append(f"  {'Timeouts':.<40} {pat['waypoints_timeout']}")
        reporte.append(f"  {'Escaneos':.<40} {pat['escaneos_completados']}")
        reporte.append("")

        # Mejoras sugeridas
        reporte.append("=" * 70)
        reporte.append("MEJORAS SUGERIDAS")
        reporte.append("=" * 70)
        mejoras = self.sugerir_mejoras()
        if mejoras:
            for m in mejoras:
                reporte.append(f"\n[{m['severidad']}] {m['area']}")
                reporte.append(f"  Problema: {m['problema']}")
                reporte.append(f"  Solución: {m['recomendacion']}")
        else:
            reporte.append("✓ No hay mejoras críticas sugeridas.")

        reporte.append("\n" + "=" * 70)
        return "\n".join(reporte)


def main():
    parser = argparse.ArgumentParser(description='Analiza datos de navegación')
    parser.add_argument('--session', required=True, help='ID de sesión (ej: 20260919_143000)')
    parser.add_argument('--output', type=Path, default=None, help='Directorio de salida')
    parser.add_argument('--guardar-reporte', action='store_true', help='Guardar reporte en archivo')

    args = parser.parse_args()

    analizador = AnalizadorNavegacion(args.session, args.output)
    reporte = analizador.generar_reporte()

    print(reporte)

    if args.guardar_reporte:
        out_file = analizador.output_dir / f"reporte_{args.session}.txt"
        out_file.write_text(reporte)
        print(f"\n✓ Reporte guardado en {out_file}")


if __name__ == '__main__':
    main()
