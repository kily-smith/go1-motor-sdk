"""Go1-style salvaged motor: 34/78-byte A1-family protocol, rotor-side units."""
from dataclasses import dataclass
import math
import struct
import time

BAUD = 4_800_000
GEAR_RATIO = 6.33
BRAKE, FOC = 0, 10
TX_SIZE, RX_SIZE = 34, 78

def queryGearRatio():
    """Return nominal rotor/output ratio; commands and feedback remain rotor-side."""
    return GEAR_RATIO

@dataclass
class MotorCmd:
    id: int = 0
    mode: int = BRAKE
    tau: float = 0.0
    dq: float = 0.0
    q: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

@dataclass
class MotorData:
    motor_id: int = 0
    mode: int = 0
    temp: int = 0
    merror: int = 0
    tau: float = 0.0
    dq: float = 0.0
    q: float = 0.0
    LW: float = 0.0
    tau_raw: int = 0
    dq_raw: int = 0
    q_raw: int = 0
    correct: bool = False

def crc32(data: bytes) -> int:
    if len(data) % 4: raise ValueError('CRC length must be a multiple of four')
    crc = 0xffffffff
    for (word,) in struct.iter_unpack('<I', data):
        crc ^= word
        for _ in range(32): crc = ((crc << 1) ^ (0x04c11db7 if crc & 0x80000000 else 0)) & 0xffffffff
    return crc

def _scale(value, factor, lo, hi):
    x = value * factor
    if not math.isfinite(x) or not lo <= x <= hi: raise ValueError('command outside protocol range')
    return int(x)

def pack(cmd: MotorCmd) -> bytes:
    if type(cmd.id) is not int or not 0 <= cmd.id <= 14 or cmd.mode not in (BRAKE, FOC): raise ValueError('invalid ID or mode')
    if cmd.mode == BRAKE and any((cmd.tau,cmd.dq,cmd.q,cmd.kp,cmd.kd)): raise ValueError('stop requires zero control fields')
    t=_scale(cmd.tau,256,-32768,32767); w=_scale(cmd.dq,128,-32768,32767)
    q=_scale(cmd.q,16384/(2*math.pi),-2147483648,2147483647)
    kp=_scale(cmd.kp,2048,0,32767); kd=_scale(cmd.kd,1024,0,32767)
    b=bytearray(TX_SIZE); b[:6]=bytes((0xfe,0xee,cmd.id,0xba,cmd.mode,0xff));b[24]=2
    struct.pack_into('<hhihh',b,12,t,w,q,kp,kd);struct.pack_into('<I',b,30,crc32(b[:28]));return bytes(b)

def unpack(buf: bytes, expected_id: int) -> MotorData:
    if type(expected_id) is not int or not 0 <= expected_id <= 14 or len(buf)!=RX_SIZE or buf[:3]!=bytes((0xfe,0xee,expected_id)): raise ValueError('invalid feedback header, length or ID')
    if crc32(buf[:72]) != struct.unpack_from('<I',buf,74)[0]: raise ValueError('feedback CRC mismatch')
    tau,dq=struct.unpack_from('<hh',buf,12);q=struct.unpack_from('<i',buf,30)[0]
    return MotorData(buf[2],buf[4],struct.unpack_from('<b',buf,6)[0],buf[7],tau/256,dq/128,q*2*math.pi/16384,struct.unpack_from('<f',buf,16)[0],tau,dq,q,True)

class Parser:
    def __init__(self): self.buffer=bytearray()
    def feed(self, chunk, expected_id):
        self.buffer.extend(chunk)
        while True:
            i=self.buffer.find(b'\xfe\xee')
            if i<0:
                self.buffer[:]=self.buffer[-1:] if self.buffer.endswith(b'\xfe') else b''
                return None
            del self.buffer[:i]
            if len(self.buffer)<RX_SIZE: return None
            try: result=unpack(bytes(self.buffer[:RX_SIZE]),expected_id)
            except ValueError: del self.buffer[0];continue
            del self.buffer[:RX_SIZE];return result

class SerialPort:
    def __init__(self, port='/dev/ttyUSB0', baudrate=BAUD):
        import serial
        self.serial=serial.Serial(port,baudrate,timeout=0,write_timeout=0.05,rtscts=False,dsrdtr=False,exclusive=True)
    def close(self): self.serial.close()
    def __enter__(self): return self
    def __exit__(self,*_): self.close()
    def sendRecv(self, cmd, data=None, timeout=0.05):
        if timeout<=0: raise ValueError('timeout must be positive')
        if data is not None: data.correct=False
        wire=pack(cmd); self.serial.reset_input_buffer(); deadline=time.monotonic()+timeout
        if self.serial.write(wire)!=len(wire): raise IOError('short serial write')
        parser=Parser()
        while time.monotonic()<deadline:
            self.serial.timeout=max(0,deadline-time.monotonic())
            chunk=self.serial.read(max(1,self.serial.in_waiting))
            if chunk:
                result=parser.feed(chunk,cmd.id)
                if result is not None:
                    if data is not None: data.__dict__.update(result.__dict__); return True
                    return result
        raise TimeoutError('no valid motor response before deadline')
