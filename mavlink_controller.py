import time
import sys
import csv
import datetime
import math

from pymavlink import mavutil

connected = False
start_mode = 'MANUAL'
end_mode = 'HOLD'

class MainController:
    def __init__(self):
        self.servo_yaw = 0
        self.servo_gimbal_tilt = 0
        self.pitch_compensation = 0
        self.start_aiming = 0
        self.fire_charge = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.wind_dir = 0
        self.wind_spd = 0
        self.distance = 0
        self.air_pressure = 0

    def arm(self):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,0,0,0,0,0,0)
        return

    def disarm(self):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,0,0,0,0,0,0)
        return

    def set_mode(self,mode):
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        return

    def set_servo_pwm(self,servo_n, microseconds):
        servo_msg = master.mav.command_long_encode(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            servo_n,  # servo instance, offset by 8 MAIN outputs
            microseconds, # PWM pulse-width
            0,0,0,0,0     # unused parameters
        )
        master.mav.send(servo_msg)
        return

    def read_mavlink_values(self):
        try:
            msg = master.recv_match(blocking=True)
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                servo_message = msg.to_dict()
                #print(servo_message)
                self.servo_yaw = servo_message["servo1_raw"]
                self.servo_gimbal_tilt = servo_message["servo2_raw"]
                self.pitch_compensation = servo_message["servo3_raw"]
                self.start_aiming = servo_message["servo4_raw"]
                self.fire_charge = servo_message["servo5_raw"]
                self.air_pressure = servo_message["servo8_raw"]
                #print(servo_yaw, servo_gimbal_tilt)
            elif msg.get_type() == 'ATTITUDE':
                att_message = msg.to_dict()
                self.roll = format(math.degrees(att_message["roll"]), '.1f')
                self.pitch = format(math.degrees(att_message["pitch"]),'.1f')
                self.yaw = format(math.degrees(att_message["yaw"]), '.1f')
            elif msg.get_type() == 'WIND':
                wind_message = msg.to_dict()
                self.wind_dir = format(wind_message["direction"],'.1f')
                self.wind_spd = format(wind_message["speed"],'.1f')
            elif msg.get_type() == 'RANGEFINDER':
                rngfndr_message = msg.to_dict()
                self.distance = format(rngfndr_message["distance"], '.2f')
            #print(self.servo_yaw, self.servo_gimbal_tilt, self.pitch_compensation,
            #      self.start_aiming, self.fire_charge, self.roll, self.pitch, self.yaw,
            #      self.wind_dir, self.wind_spd, self.distance, self.air_pressure)
        except Exception as e:
            print(f"Error reading MAVLink values: {e}")

        return (self.servo_yaw, self.servo_gimbal_tilt, self.pitch_compensation,
                  self.start_aiming, self.fire_charge, self.roll, self.pitch, self.yaw,
                  self.wind_dir, self.wind_spd, self.distance, self.air_pressure)
        
main = MainController()

while not connected:
    try:
        master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
        #master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
        print("Attempting connection")
        if master.wait_heartbeat():
            connected = True
            print("Connected")
            main.set_mode(start_mode)
            time.sleep(2)
            break
    except:
        time.sleep(1)
        print("Waiting for connection")

try:
    while True:
        values = main.read_mavlink_values()
        if values:
            (servo_yaw, servo_gimbal_tilt, pitch_compensation, start_aiming,
             fire_charge, roll, pitch, yaw, wind_dir, wind_spd, distance, air_pressure) = values
            
            # Print each variable
            print(f"Servo Yaw: {servo_yaw}")
            print(f"Servo Gimbal Tilt: {servo_gimbal_tilt}")
            print(f"Pitch Compensation: {pitch_compensation}")
            print(f"Start Aiming: {start_aiming}")
            print(f"Fire Charge: {fire_charge}")
            print(f"Roll: {roll}")
            print(f"Pitch: {pitch}")
            print(f"Yaw: {yaw}")
            print(f"Wind Direction: {wind_dir}")
            print(f"Wind Speed: {wind_spd}")
            print(f"Distance: {distance}")
            print(f"Air Pressure: {air_pressure}")
        else:
            print("No values returned.")
            
        
            
        #if start_aiming > 1600:
        #    set_servo_pwm(6, 1750)
        #else:
        #    set_servo_pwm(6, 1500)
                
except:
    main.disarm()
    #main.set_servo_pwm(6, 1500)
    main.set_mode(end_mode)
    print("Script interrupted. Closing file...")