#!/usr/bin/env python3
"""Send one stop frame and print feedback; requires a connected motor."""
import argparse
from go1_motor_sdk import MotorCmd, SerialPort, BRAKE
p=argparse.ArgumentParser();p.add_argument('--port',default='/dev/ttyUSB0');p.add_argument('--id',type=int,required=True);a=p.parse_args()
with SerialPort(a.port) as serial:
    print(serial.sendRecv(MotorCmd(id=a.id,mode=BRAKE)))
