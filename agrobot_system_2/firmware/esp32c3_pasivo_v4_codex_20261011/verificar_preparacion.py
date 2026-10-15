"""Read-only preparation report. Never opens motor, BLE or UDP control channels."""
import json, pathlib, shutil, subprocess, hashlib
root=pathlib.Path(__file__).resolve().parent
def run(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=30)
 return {'exit':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
report={'scope':'read-only; no flash or actuator commands','files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('*.ino')},'usb_ports':[str(p) for pattern in ['ttyACM*','ttyUSB*'] for p in pathlib.Path('/dev').glob(pattern)],'network':run(['ip','-brief','address']),'disk':run(['df','-h',str(root)]),'backend':run(['pgrep','-af','jetson/backend/jetson_server.py'])}
(root/'preparacion.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
