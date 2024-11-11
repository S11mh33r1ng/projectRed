import time
import sys
import csv
import datetime
import math

from pymavlink import mavutil
from scipy.optimize import fsolve

start_mode = 'MANUAL'
end_mode = 'HOLD'
master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

class TargetController:
    def __init__(self):
        self.g_constant = 9.8065
        self.camera_height = 2
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
        self.gimbal_zero_pwm = 1515
        self.gimbal_max_up_pwm = 1295
        self.gimbal_min_down_pwm = 1705
        self.gimbal_max_angle = 20
        self.gimbal_angle = 0
        self.target_height = 0
        self.initial_velocity = 100
        self.connected = False

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
        except Exception as e:
            print(f"Error reading MAVLink values: {e}")

        return (self.servo_yaw, self.servo_gimbal_tilt, self.pitch_compensation,
                  self.start_aiming, self.fire_charge, self.roll, self.pitch, self.yaw,
                  self.wind_dir, self.wind_spd, self.distance, self.air_pressure)
    
    def calculate_gimbal_angle(self):
        if self.servo_gimbal_tilt < self.gimbal_zero_pwm:
            up_span = self.gimbal_zero_pwm - self.gimbal_max_up_pwm
            single_step_up = up_span/self.gimbal_max_angle
            self.gimbal_angle = format((self.gimbal_zero_pwm - self.servo_gimbal_tilt)/single_step_up,'.1f')
            return(self.gimbal_angle)
        if self.servo_gimbal_tilt == self.gimbal_zero_pwm:
            self.gimbal_angle = 0
            return(self.gimbal_angle)
        if self.servo_gimbal_tilt > self.gimbal_zero_pwm:
            down_span = self.gimbal_min_down_pwm - self.gimbal_zero_pwm
            single_step_down = down_span/self.gimbal_max_angle
            self.gimbal_angle = format(((self.servo_gimbal_tilt - self.gimbal_zero_pwm)/single_step_down)*-1,'.1f')
            return(self.gimbal_angle)
        
    def calculate_target_height(self, distance, camera_height):
        angle_radians = math.radians(float(self.calculate_gimbal_angle()))
        height_difference = distance * math.tan(angle_radians)
        target_height = camera_height + height_difference
        return target_height
    
    def projectile_motion(self, theta, x, y, v, v_wind, dir_wind):
        theta_rad = math.radians(theta)
        dir_wind_rad = math.radians(dir_wind)
        v_eff = v + v_wind * math.cos(dir_wind_rad)
        t_flight = (v_eff * math.sin(theta_rad) + math.sqrt((v_eff * math.sin(theta_rad))**2 + 2*self.g_constant*y)) / self.g_constant
        x_adjusted = x - v_wind * math.sin(dir_wind_rad) * t_flight
        term1 = x_adjusted * math.tan(theta_rad)
        term2 = (self.g_constant*x_adjusted**2)/(2*v_eff**2*math.cos(theta_rad)**2)
        return term1-term2-y
    
    def calculate_yaw_angle(self, v, v_wind, dir_wind, t_flight, theta):
        if dir_wind == 90 or dir_wind == 270:
            theta_rad = math.radians(theta)
            v_hor = v * math.cos(theta_rad)
            yaw_angle_rad = math.atan(v_wind * math.sin(math.radians(dir_wind)) * t_flight / v_hor)
            return math.degrees(yaw_angle_rad)
        return 0.0
    
    def find_launch_angle(self, x, y, v, v_wind, dir_wind):
        initial_guess = 45.0
        angle_solution = fsolve(self.projectile_motion, initial_guess, args=(x, y, v, v_wind, dir_wind))
        theta = angle_solution[0]
        theta_rad = math.radians(theta)
        v_eff = v + v_wind * math.cos(math.radians(dir_wind))
        t_flight = (v_eff * math.sin(theta_rad) + math.sqrt((v_eff * math.sin(theta_rad))**2 + 2 * self.g_constant * y)) / self.g_constant
        yaw_angle = self.calculate_yaw_angle(v, v_wind, dir_wind, t_flight, theta)
        return (float(format(theta,'.1f')), yaw_angle)

class PIController:
    def __init__(self, k_p, k_i):
        self.k_p = k_p
        self.k_i = k_i
        self.integral = 0 #Accumulated error
        
    def calculate_pwm_signal(self, current_angle, target_angle, delta_time):
        PWM_STOP = 1500
        PWM_FULL_DOWN = 1100
        PWM_FULL_UP = 1900
        
        error = float(target_angle) - float(current_angle)
        
        proportional = self.k_p * error
        
        self.integral += error * delta_time
        integral = self.k_i * self.integral
        
        control_signal = proportional + integral
        
        if control_signal > 0:
            pwm_signal = PWM_STOP + min(control_signal, PWM_FULL_UP - PWM_STOP)
        elif control_signal < 0:
            pwm_signal = PWM_STOP - min(abs(control_signal), PWM_STOP - PWM_FULL_DOWN)
        else:
            pwm_signal = PWM_STOP
            
        return int(pwm_signal)
    
    


def main():
    target = TargetController()
    yaw_controller = PIController(k_p=2.0, k_i=0.1)
    elevation_controller = PIController(k_p=2.0, k_i=0.1)
    delta_time = 0.1
    
    while not target.connected:
        try:
            #master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
            #master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
            print("Attempting connection")
            if master.wait_heartbeat():
                target.connected = True
                print("Connected")
                target.set_mode(start_mode)
                time.sleep(2)
                break
        except:
            time.sleep(1)
            print("Waiting for connection")

    try:
        while True:
            values = target.read_mavlink_values()
            if values:
                (servo_yaw, servo_gimbal_tilt, pitch_compensation, start_aiming,
                 fire_charge, roll, pitch, yaw, wind_dir, wind_spd, distance, air_pressure) = values
                
                # Print each variable
        #             print(f"Servo Yaw: {servo_yaw}")
        #             print(f"Servo Gimbal Tilt: {servo_gimbal_tilt}")
        #             print(f"Pitch Compensation: {pitch_compensation}")
        #             print(f"Start Aiming: {start_aiming}")
        #             print(f"Fire Charge: {fire_charge}")
        #             print(f"Roll: {roll}")
        #             print(f"Pitch: {pitch}")
        #             print(f"Yaw: {yaw}")
        #             print(f"Wind Direction: {wind_dir}")
        #             print(f"Wind Speed: {wind_spd}")
        #             print(f"Distance: {distance}")
        #             print(f"Air Pressure: {air_pressure}")
            else:
                print("No values returned.")
                
            target_height = target.calculate_target_height(float(distance), target.camera_height)
            #print(target_height)
                
            if start_aiming > 1600:
                #print(distance, round(target_height,2), target.gimbal_angle)
                launch_angle = target.find_launch_angle(float(distance), float(target_height), float(target.initial_velocity), float(wind_spd), float(wind_dir))
                target_angle = launch_angle[0]
                yaw_correction = launch_angle[1]
                target_yaw = float(yaw) + float(yaw_correction)
                #print(launch_angle, target_angle, yaw_correction)
                yaw_pwm = yaw_controller.calculate_pwm_signal(yaw, target_yaw, delta_time)
                elevation_pwm = elevation_controller.calculate_pwm_signal(pitch, target_angle, delta_time)
                print(yaw_pwm, elevation_pwm)
                target.set_servo_pwm(1, yaw_pwm)
                target.set_servo_pwm(3, elevation_pwm)
                        
    except:
        target.disarm()
        #main.set_servo_pwm(6, 1500)
        target.set_mode(end_mode)
        print("Script interrupted. Closing file...")
        
if __name__ == "__main__":
    main()
