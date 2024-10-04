import time
import sys
import csv
import datetime
import math

from pymavlink import mavutil

connected = False
servo_yaw = 0
servo_gimbal_tilt = 0
pitch_compensation = 0
start_aiming = 0
fire_charge = 0
roll = 0
pitch = 0
yaw = 0
wind_dir = 0
wind_spd = 0
distance = 0
air_pressure = 0

while not connected:
    try:
        master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
        #master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
        print("Attempting connection")
        if master.wait_heartbeat:
            connected = True
            print("Connected")
            break
    except:
        time.sleep(1)
        print("Waiting for connection")

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    servo_msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n,  # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )
    master.mav.send(servo_msg)

try:
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SERVO_OUTPUT_RAW':
            servo_message = msg.to_dict()
            servo_yaw = servo_message["servo1_raw"]
            servo_gimbal_tilt = servo_message["servo2_raw"]
            pitch_compensation = servo_message["servo3_raw"]
            start_aiming = servo_message["servo4_raw"]
            fire_charge = servo_message["servo5_raw"]
            air_pressure = servo_message["servo8_raw"]
        if msg.get_type() == 'ATTITUDE':
            att_message = msg.to_dict()
            roll = format(math.degrees(att_message["roll"]), '.1f')
            pitch = format(math.degrees(att_message["pitch"]),'.1f')
            yaw = format(math.degrees(att_message["yaw"]), '.1f')
        if msg.get_type() == 'WIND':
            wind_message = msg.to_dict()
            wind_dir = format(wind_message["direction"],'.1f')
            wind_spd = format(wind_message["speed"],'.1f')
        if msg.get_type() == 'RANGEFINDER':
            rngfndr_message = msg.to_dict()
            distance = format(rngfndr_message["distance"], '.2f')
        print(servo_yaw, servo_gimbal_tilt, pitch_compensation, start_aiming, fire_charge, roll, pitch, yaw, wind_dir, wind_spd, distance, air_pressure)
        
        if start_aiming > 1600:
            set_servo_pwm(6, 1750)
        else:
            set_servo_pwm(6, 1500)
                
except:
    set_servo_pwm(6, 1500)
    print("Script interrupted. Closing file...")