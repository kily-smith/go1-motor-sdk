#!/usr/bin/env python3
"""Offline protocol and PTY integration against a captured real feedback frame."""
import os, pty, select, subprocess, sys, threading, time, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from go1_motor_sdk import MotorCmd, SerialPort, Parser, pack, unpack, FOC, queryGearRatio
ROOT=Path(__file__).resolve().parents[1]
REPLY=bytes.fromhex('feee02010000280000f838410000bdff349ec83b000000000000f7ff00005536000000000000ffffffffffffffffffffffff000000000000000000000000000000000000622d070000004693f359')
STOP=bytes.fromhex('feee01ba00ff000000000000000000000000000000000000020000000000d8106475')
class TestProtocol(unittest.TestCase):
 def test_real_capture(self):
  self.assertEqual(queryGearRatio(),6.33)
  self.assertEqual(pack(MotorCmd(id=1)),STOP)
  data=unpack(REPLY,2);self.assertEqual((data.motor_id,data.mode,data.temp,data.merror,data.q_raw),(2,0,40,0,13909))
  broken=bytearray(REPLY);broken[12]^=1
  with self.assertRaises(ValueError):unpack(broken,2)
  with self.assertRaises(ValueError):unpack(REPLY,1)
  parser=Parser();result=None
  for b in b'noise'+STOP+bytes(broken)+REPLY:
   result=parser.feed(bytes([b]),2) or result
  self.assertEqual(result.q_raw,13909)
 def test_input_validation(self):
  for cmd in (MotorCmd(id=255),MotorCmd(mode=11),MotorCmd(mode=0,dq=1),MotorCmd(mode=FOC,kp=-1),MotorCmd(mode=FOC,dq=float('nan'))):
   with self.assertRaises(ValueError):pack(cmd)
 def exchange(self,kind,respond):
  master,slave=pty.openpty();path=os.ttyname(slave);os.close(slave); received=[];errors=[]
  def peer():
   try:
    wire=bytearray();end=time.monotonic()+4
    while len(wire)<34 and time.monotonic()<end:
     ready,_,_=select.select([master],[],[],.05)
     if ready:
      try:wire.extend(os.read(master,34-len(wire)))
      except OSError:time.sleep(.01)
    received.append(bytes(wire))
    if respond and len(wire)==34:
     os.write(master,wire+b'\x12\x34'+REPLY[:15]);time.sleep(.005);os.write(master,REPLY[15:])
   except Exception as e:errors.append(e)
  th=threading.Thread(target=peer);th.start()
  try:
   if kind=='python':
    with SerialPort(path) as port:
     if respond:self.assertEqual(port.sendRecv(MotorCmd(id=2),timeout=.3).motor_id,2)
     else:
      with self.assertRaises(TimeoutError):port.sendRecv(MotorCmd(id=2),timeout=.08)
   else:
    result=subprocess.run([str(ROOT/'build/go1_stop'),path,'2'],capture_output=True,text=True,timeout=4)
    if respond:self.assertEqual(result.returncode,0,result.stderr);self.assertIn('ID=2 mode=0',result.stdout)
    else:self.assertNotEqual(result.returncode,0);self.assertIn('timeout',result.stderr)
  finally:th.join(4);os.close(master)
  self.assertFalse(errors);self.assertEqual(received,[pack(MotorCmd(id=2))])
 def test_python_pty(self):self.exchange('python',True)
 def test_cpp_pty(self):self.exchange('cpp',True)
 def test_python_timeout(self):self.exchange('python',False)
 def test_cpp_timeout(self):self.exchange('cpp',False)
if __name__=='__main__':unittest.main(verbosity=2)
