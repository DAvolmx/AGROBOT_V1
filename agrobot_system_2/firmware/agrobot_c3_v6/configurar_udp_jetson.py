"""Run ON THE JETSON: python3 configurar_udp_jetson.py /path/to/backend
Adds exact configured C3 LAN IP to UDP source allowlist. Does not start motors.
"""
import argparse
from pathlib import Path
import time
p=argparse.ArgumentParser();p.add_argument('backend',type=Path);p.add_argument('--ip',default='192.168.0.50');a=p.parse_args()
import ipaddress
ip=str(ipaddress.ip_address(a.ip))
path=a.backend/'udp_actuator_link.py';source=path.read_text()
if 'IP_C3_FIJA' not in source:
 source=source.replace('RED_AP = ipaddress.ip_network("192.168.12.0/24")',
     'RED_AP = ipaddress.ip_network("192.168.12.0/24")\nIP_C3_FIJA = ipaddress.ip_address("'+ip+'")')
 old='if ip not in RED_AP:'
 if old not in source:raise SystemExit('Version distinta; no se modifico el archivo')
 source=source.replace(old,'if ip not in RED_AP and ip != IP_C3_FIJA:')
 compile(source,str(path),'exec')
 path.with_suffix('.py.backup_'+time.strftime('%Y%m%d_%H%M%S')).write_text(path.read_text())
 path.write_text(source)
 print('UDP acepta IP C3 exacta '+ip+' y red AP original. Reiniciar backend con robot parado.')
else:print('Ya existe IP_C3_FIJA; revisar que coincida con la configuracion del firmware.')
