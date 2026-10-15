"""Separate BLE sensor collector, feeds existing UDP5001 receiver.
Sensor-only, never sends actuator or LoRa commands.
"""
import asyncio,json,socket
from bleak import BleakScanner,BleakClient
from lora_sensor_codec import SERVICE,CHAR,Fragments
async def main():
 sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.setblocking(False)
 while True:
  try:
   d=await BleakScanner.find_device_by_filter(lambda dev,ad:SERVICE in [x.lower() for x in ad.service_uuids],timeout=8)
   if d is None:await asyncio.sleep(15);continue
   parser=Fragments()
   def receive(_,data):
    sample=parser.feed(data)
    if sample:
     try:sock.sendto(json.dumps(sample).encode(),('127.0.0.1',5001))
     except OSError:pass
   async with BleakClient(d,timeout=12) as client:
    await client.start_notify(CHAR,receive)
    print('BLE sensores conectado',flush=True)
    while client.is_connected:await asyncio.sleep(1)
  except Exception as e:print('BLE sensores:',type(e).__name__,flush=True)
  await asyncio.sleep(10)
if __name__=='__main__':asyncio.run(main())
