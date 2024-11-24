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
#master = mavutil.mavlink_connection("/dev/serial0", baud=921600)

class TargetController:
    def __init__(self):
        self.g_constant = 9.8065
        self.camera_height = 2
        self.servo_yaw = 0
        self.servo_gimbal_tilt = 0
        self.pitch_compensation = 0
        self.start_aiming_command = 0
        self.fire_charge_input = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.wind_dir = 0
        self.wind_spd = 0
        self.distance = 0
        self.air_pressure = 0
        self.charge_initial_velocity = 0
        self.charge_velocity_compensation = 0
        self.charge_type = 0
        self.gimbal_zero_pwm = 1515
        self.gimbal_max_up_pwm = 1295
        self.gimbal_min_down_pwm = 1705
        self.gimbal_max_angle = 20
        self.gimbal_angle = 0
        self.target_height = 0
        self.yaw_channel = 1
        self.elev_channel = 3
        self.trigger_first_channel = 6
        self.trigger_second_channel = 7
        self.corrected_velocity_channel = 12
        self.connected = False
        self.previous_factor = 1500

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
                self.start_aiming_command = servo_message["servo4_raw"]
                self.fire_charge_input = servo_message["servo5_raw"]
                self.air_pressure = servo_message["servo8_raw"]
                self.charge_initial_velocity = servo_message["servo9_raw"]
                self.charge_velocity_compensation = servo_message["servo10_raw"]
                self.charge_type = servo_message["servo11_raw"]
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
                  self.start_aiming_command, self.fire_charge_input, self.roll, self.pitch, self.yaw,
                  self.wind_dir, self.wind_spd, self.distance, self.air_pressure, self.charge_initial_velocity,
                  self.charge_velocity_compensation, self.charge_type)
    
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
    
    def calculate_exit_velocity(self, initial_velocity, correction_factor_raw):
        if correction_factor_raw == 1500:
            if self.previous_factor != correction_factor_raw:
                self.set_servo_pwm(self.corrected_velocity_channel, initial_velocity)
            self.previous_factor = correction_factor_raw
            return initial_velocity
        elif correction_factor_raw < 1500:
            correction_pct = (1500 - correction_factor_raw)/1000
            corrected_velocity = int(initial_velocity - (initial_velocity*correction_pct))
            if self.previous_factor != correction_factor_raw:
                self.set_servo_pwm(self.corrected_velocity_channel, corrected_velocity)
            self.previous_factor = correction_factor_raw
            return corrected_velocity
        elif correction_factor_raw > 1500:
            correction_pct = (correction_factor_raw - 1500)/1000
            corrected_velocity = int(initial_velocity + (initial_velocity*correction_pct))
            if self.previous_factor != correction_factor_raw:
                self.set_servo_pwm(self.corrected_velocity_channel, corrected_velocity)
            self.previous_factor = correction_factor_raw
            return corrected_velocity
        

class PIController:
    def __init__(self, k_p, k_i, max_integral = float('inf'), decay_factor = 1.0):
        self.k_p = k_p
        self.k_i = k_i
        self.max_integral = max_integral
        self.decay_factor = decay_factor
        self.integral = 0 #Accumulated error
        
    def calculate_pwm_signal(self, current_angle, target_angle, delta_time):
        PWM_STOP = 1500
        PWM_FULL_DOWN = 1100
        PWM_FULL_UP = 1900
        
        error = float(target_angle) - float(current_angle)
        
        proportional = self.k_p * error
        
        self.integral += error * delta_time
        self.integral *= self.decay_factor
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
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
    yaw_controller = PIController(k_p=10.0, k_i=0.2, max_integral=50, decay_factor=0.99)
    elevation_controller = PIController(k_p=20.0, k_i=0.2, max_integral=50, decay_factor=0.99)
    delta_time = 0.1
    start_aiming_flag = False
    target_attitude_reached_flag = False
    ready_to_fire_flag = False
    fire_charge_flag = False
    
    while not target.connected:
        try:
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
                (servo_yaw, servo_gimbal_tilt, pitch_compensation, start_aiming_command,
                 fire_charge_input, roll, pitch, yaw, wind_dir, wind_spd, distance, air_pressure,
                 charge_initial_velocity, charge_velocity_compensation, charge_type) = values
            else:
                print("No values returned.")
                
            target_height = target.calculate_target_height(float(distance), target.camera_height)
            #print(target_height)
            
            initv = target.calculate_exit_velocity(charge_initial_velocity, charge_velocity_compensation)
            #print(initv)
            
            if start_aiming_command > 1600 and not ready_to_fire_flag:
                start_aiming_flag = True
            
            while start_aiming_flag == True and ready_to_fire_flag == False and fire_charge_flag == False:
                #print(distance, round(target_height,2), target.gimbal_angle)
                target_attitude_reached_flag = False
                ready_to_fire_flag = False
                fire_charge_flag = False
                launch_angle = target.find_launch_angle(float(distance), float(target_height), initv, float(wind_spd), float(wind_dir))
                target_angle = launch_angle[0]
                yaw_correction = launch_angle[1]
                target_yaw = float(yaw) + float(yaw_correction)
                print(pitch, target_angle, yaw_correction)
                yaw_pwm = yaw_controller.calculate_pwm_signal(yaw, target_yaw, delta_time)
                elevation_pwm = elevation_controller.calculate_pwm_signal(pitch, target_angle, delta_time)
                print(yaw_pwm, elevation_pwm)
                print(elevation_controller.integral)
                target.set_servo_pwm(target.yaw_channel, yaw_pwm)
                target.set_servo_pwm(target.elev_channel, elevation_pwm)
                values = target.read_mavlink_values()
                if values:
                    (servo_yaw, servo_gimbal_tilt, pitch_compensation, start_aiming_command,
                     fire_charge_input, roll, pitch, yaw, wind_dir, wind_spd, distance, air_pressure,
                     charge_initial_velocity, charge_velocity_compensation, charge_type) = values
                    if float(pitch) == float(target_angle) and yaw_pwm == 1500 and elevation_pwm == 1500:
                        target.set_servo_pwm(target.yaw_channel, yaw_pwm)
                        target.set_servo_pwm(target.elev_channel, elevation_pwm)
                        target_attitude_reached_flag = True
                        if air_pressure > 0:
                            target.arm()
                            start_aiming_flag = False
                            ready_to_fire_flag = True
                            break
                        
            if start_aiming_command > 1600 and fire_charge_input > 1600 and ready_to_fire_flag:
                fire_charge_flag = True
               
            if target_attitude_reached_flag and ready_to_fire_flag and fire_charge_flag:
                target.set_servo_pwm(target.trigger_first_channel, 1100) #trigger actuator in
                target.set_servo_pwm(target.trigger_second_channel, 1100) #trigger actuator in
                time.sleep(5)
                target.set_servo_pwm(target.trigger_first_channel, 1500) #trigger actuator out
                target.set_servo_pwm(target.trigger_second_channel, 1500) #trigger actuator out
                time.sleep(5)
                target_attitude_reached_flag = False
                ready_to_fire_flag = False
                start_aiming_flag = False
                target.disarm()
                break
            
            #add angle limitations for elevation (clamping)
            #create a method for shutting down the computer
            #make automatic startup
                        
    except:
        target.disarm()
        #main.set_servo_pwm(6, 1500)
        target.set_mode(end_mode)
        print("Script interrupted. Closing file...")
        
if __name__ == "__main__":
    main()
