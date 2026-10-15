import struct,time,math
FORMAT='<BBIII6hHhHBB'
SERVICE='b0e1b000-7f3c-4a5e-9d21-6b8e2c4f1a00'
CHAR='b0e1b002-7f3c-4a5e-9d21-6b8e2c4f1a00'
def crc8(b):
 c=0
 for v in b:
  c^=v
  for _ in range(8):c=((c<<1)^7)&255 if c&128 else (c<<1)&255
 return c
class Fragments:
 def __init__(self):self.pending={}
 def feed(self,data):
  now=time.monotonic()
  self.pending={k:v for k,v in self.pending.items() if now-v[0]<.25}
  if len(data)<5 or data[0]!=0xB6:return None
  key=data[1]|data[2]<<8;part=data[3]
  if part>2 or len(data)!=(20 if part<2 else 6):return None
  if key not in self.pending:
   if len(self.pending)>=4:self.pending.clear()
   self.pending[key]=(now,{})
  parts=self.pending[key][1];parts[part]=bytes(data[4:])
  if len(parts)!=3:return None
  raw=b''.join(parts[i] for i in range(3));del self.pending[key]
  if len(raw)!=34 or crc8(raw[:-1])!=raw[-1]:return None
  v,flags,boot,seq,ms,*a=struct.unpack(FORMAT,raw)
  if v!=3 or seq&65535!=key:return None
  imu=a[:6];co2,temp,humidity,reset,_=a[6:]
  d=dict(zip(('accelX','accelY','accelZ','gyroX','gyroY','gyroZ'),[x/(1000 if i<3 else 100) for i,x in enumerate(imu)]))
  d.update(sensor_v=3,boot_id=boot,seq=seq,sample_ms=ms,uptime_ms=ms,reset_reason=reset,imu_ok=bool(flags&1),env_ok=bool(flags&2),co2=co2,tempSCD=temp/100,humidity=humidity/100)
  return d
class SampleGate:
 def __init__(self):self.boot=None;self.seq=None;self.retired=[];self.rejected=0
 def accept(self,d):
  try:
   if d.get('sensor_v')!=3 or d.get('imu_ok') is not True:return False
   b,q=d['boot_id'],d['seq']
   if type(b)is not int or type(q)is not int or not 0<b<2**32 or not 0<=q<2**32:return False
   if not all(isinstance(d[k],(int,float)) and math.isfinite(d[k]) for k in ('accelX','accelY','accelZ','gyroX','gyroY','gyroZ')):return False
   if b!=self.boot:
    if b in self.retired:return False
    if self.boot is not None:self.retired=(self.retired+[self.boot])[-32:]
    self.boot=b;self.seq=q;return True
   delta=(q-self.seq)&0xffffffff
   if not 0<delta<0x80000000:self.rejected+=1;return False
   self.seq=q;return True
  except (KeyError,TypeError,ValueError):return False
