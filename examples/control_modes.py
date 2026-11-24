"""A1-style command templates. This file does not open a serial port or move a motor.
Parameters are illustrative only; determine limits, transmission and zero on your rig.
"""
from go1_motor_sdk import MotorCmd, FOC

def examples(motor_id, q_rotor, dq_rotor):
    # Raw protocol kp/kd coefficients, not calibrated output-side joint gains.
    return {
        'position': MotorCmd(id=motor_id,mode=FOC,q=q_rotor,kp=0.01,kd=0.1),
        'velocity': MotorCmd(id=motor_id,mode=FOC,dq=dq_rotor,kd=0.1),
        'damping': MotorCmd(id=motor_id,mode=FOC,kd=0.1),
        'torque': MotorCmd(id=motor_id,mode=FOC,tau=0.01),
        'zero_torque': MotorCmd(id=motor_id,mode=FOC),
        'hybrid': MotorCmd(id=motor_id,mode=FOC,q=q_rotor,dq=dq_rotor,kp=0.01,kd=0.1,tau=0.01),
    }
