import time
import sys
import csv
import datetime
import math

from pymavlink import mavutil
from scipy.optimize import fsolve

start_mode = 'MANUAL'
end_mode = 'HOLD'
#master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
master = mavutil.mavlink_connection("/dev/serial0", baud=921600)

class TargetController:
    def __init__(self):
        self.g_constant = 9.8065
        self.camera_height = 1.715
        self.servo_yaw_out = 1500
        self.servo_yaw_in = 1500
        self.servo_gimbal_tilt = 0
        self.pitch_compensation_out = 1500
        self.servo_pitch_in = 1500
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
        self.start_aiming_channel = 4
        self.trigger_channel = 5
        self.connected = False
        self.over_limit_flag = False
        self.previous_factor = 1500
        self.max_elevation_angle = 40
        self.min_elevation_angle = -5
        self.initial_guess = 0
        self.previous_yaw_pwm = None
        self.previous_elev_pwm = None
        self.triggering_time = 5

    def arm(self):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,0,0,0,0,0,0)
        master.motors_armed_wait()
        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, "READY TO FIRE!".encode())
        return

    def disarm(self):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,0,0,0,0,0,0)
        master.motors_disarmed_wait()
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
        #print(servo_msg)
        return
    
    def set_elev_servo_pwm(self,servo_n, microseconds,current_angle):
        if float(current_angle) < self.min_elevation_angle:
            microseconds = 1500 if microseconds > 1500 else microseconds
        elif float(current_angle) > self.max_elevation_angle:
            microseconds = 1500 if microseconds < 1500 else microseconds
        
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
    
    def update_servo_pwm(self, servo_n, microseconds, previous_pwm):
        if microseconds != previous_pwm:
            self.set_servo_pwm(servo_n, microseconds)
            return microseconds
        return previous_pwm
    
    def update_elev_servo_pwm(self, servo_n, microseconds, current_angle, previous_pwm):
        if microseconds != previous_pwm or float(current_angle) < self.min_elevation_angle or float(current_angle) > self.max_elevation_angle:
            self.set_elev_servo_pwm(servo_n, microseconds, current_angle)
            return microseconds
        return previous_pwm

    def read_mavlink_values(self):
        try:
            msg = master.recv_match(blocking=True)
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                servo_message = msg.to_dict()
                #print(servo_message)
                self.servo_yaw_out = servo_message["servo1_raw"]
                self.servo_gimbal_tilt = servo_message["servo2_raw"]
                self.pitch_compensation_out = servo_message["servo3_raw"]
                self.start_aiming_command = servo_message["servo4_raw"]
                self.fire_charge_input = servo_message["servo5_raw"]
                self.air_pressure = servo_message["servo8_raw"]
                self.charge_initial_velocity = servo_message["servo9_raw"]
                self.charge_velocity_compensation = servo_message["servo10_raw"]
                self.charge_type = servo_message["servo11_raw"]
                self.servo_yaw_in = servo_message["servo15_raw"]
                self.servo_pitch_in = servo_message["servo16_raw"]
                #print(servo_yaw, servo_gimbal_tilt)
            elif msg.get_type() == 'ATTITUDE':
                att_message = msg.to_dict()
                self.roll = format(math.degrees(att_message["roll"]), '.0f')
                self.pitch = format(math.degrees(att_message["pitch"]),'.0f')
                self.yaw = format(math.degrees(att_message["yaw"]), '.0f')
            elif msg.get_type() == 'WIND':
                wind_message = msg.to_dict()
                self.wind_dir = format(wind_message["direction"],'.1f')
                self.wind_spd = format(wind_message["speed"],'.1f')
            elif msg.get_type() == 'RANGEFINDER':
                rngfndr_message = msg.to_dict()
                self.distance = format(rngfndr_message["distance"], '.2f')
        except Exception as e:
            print(f"Error reading MAVLink values: {e}")

        return (self.servo_yaw_out, self.servo_gimbal_tilt, self.pitch_compensation_out,
                  self.start_aiming_command, self.fire_charge_input, self.roll, self.pitch, self.yaw,
                  self.wind_dir, self.wind_spd, self.distance, self.air_pressure, self.charge_initial_velocity,
                  self.charge_velocity_compensation, self.charge_type, self.servo_yaw_in, self.servo_pitch_in)
    
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
    
        # Resolve wind into components
        v_wind_x = v_wind * math.sin(dir_wind_rad)  # Lateral wind component
        v_wind_y = v_wind * math.cos(dir_wind_rad)  # Tail/head wind component
    
        # Adjusted initial velocity components
        v_eff = v + v_wind_y
    
        # Flight time considering vertical motion
        t_flight = (v_eff * math.sin(theta_rad) + math.sqrt((v_eff * math.sin(theta_rad))**2 + 2 * self.g_constant * y)) / self.g_constant
    
        # Adjusted horizontal displacement
        x_adjusted = x - v_wind_x * t_flight
    
        # Calculate the vertical displacement equation
        term1 = x_adjusted * math.tan(theta_rad)
        term2 = (self.g_constant * x_adjusted ** 2) / (2 * v_eff ** 2 * math.cos(theta_rad) ** 2)
    
        return term1 - term2 - y
    
    def calculate_yaw_angle(self, v, v_wind, dir_wind, t_flight, theta):
        theta_rad = math.radians(theta)
        dir_wind_rad = math.radians(dir_wind)
    
        # Resolve wind into components
        v_wind_x = v_wind * math.sin(dir_wind_rad)  # Lateral wind component
        v_wind_y = v_wind * math.cos(dir_wind_rad)  # Tail/head wind component
    
        # Effective horizontal velocity of the projectile
        v_hor = v * math.cos(theta_rad)
    
        # Adjust yaw correction based on wind components
        if v_hor != 0:
            yaw_angle_rad = math.atan(v_wind_x * t_flight / v_hor)
            yaw_correction = math.degrees(yaw_angle_rad)
        else:
            yaw_correction = 0.0
    
        return yaw_correction
    
    def find_launch_angle(self, x, y, v, v_wind, dir_wind):
        self.over_limit_flag = False
        angle_solution = fsolve(self.projectile_motion, self.initial_guess, args=(x, y, v, v_wind, dir_wind))
        theta = angle_solution[0]
        if theta > self.max_elevation_angle or theta < self.min_elevation_angle:
            self.over_limit_flag = True
        theta = max(min(theta, self.max_elevation_angle), self.min_elevation_angle)
        theta_rad = math.radians(theta)
        v_eff = v + v_wind * math.cos(math.radians(dir_wind))
        t_flight = (v_eff * math.sin(theta_rad) + math.sqrt((v_eff * math.sin(theta_rad))**2 + 2 * self.g_constant * y)) / self.g_constant
        yaw_angle = self.calculate_yaw_angle(v, v_wind, dir_wind, t_flight, theta)
        return (float(format(theta,'.0f')), yaw_angle, self.over_limit_flag)
    
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
        PWM_FULL_DOWN = 1900
        PWM_FULL_UP = 1100
        
        error = float(target_angle) - float(current_angle)
        
        proportional = self.k_p * error
        
        self.integral += error * delta_time
        self.integral *= self.decay_factor
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        integral = self.k_i * self.integral
        
        control_signal = proportional + integral
        
        if control_signal > 0:
            pwm_signal = PWM_STOP - min(control_signal, PWM_STOP - PWM_FULL_UP)
        elif control_signal < 0:
            pwm_signal = PWM_STOP + min(abs(control_signal), PWM_FULL_DOWN - PWM_STOP)
        else:
            pwm_signal = PWM_STOP
            
        return int(pwm_signal)

def main():
    target = TargetController()
    yaw_controller = PIController(k_p=10.0, k_i=0.2, max_integral=50, decay_factor=0.99)
    elevation_controller = PIController(k_p=17.0, k_i=5.0, max_integral=50, decay_factor=0.99)
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
                target.disarm()
                time.sleep(1)
                target.set_mode(start_mode)
                time.sleep(1)
                target.set_servo_pwm(target.trigger_first_channel, 1500) #trigger actuator out
                target.set_servo_pwm(target.trigger_second_channel, 1500) #trigger actuator out
                break
        except:
            time.sleep(1)
            print("Waiting for connection")

    try:
        while target.connected:
            try:
                values = target.read_mavlink_values()
                (servo_yaw_out, servo_gimbal_tilt, pitch_compensation_out, start_aiming_command,
                 fire_charge_input, roll, pitch, yaw, wind_dir, wind_spd, distance, air_pressure,
                 charge_initial_velocity, charge_velocity_compensation, charge_type, yaw_in, pitch_in) = values
                initv = target.calculate_exit_velocity(charge_initial_velocity, charge_velocity_compensation)
            except:
                target.connected = False
                break
            
            if start_aiming_flag == True and ready_to_fire_flag == False and fire_charge_flag == False and target_attitude_reached_flag == False:
                master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, "AIMING...".encode())
                target_height = target.calculate_target_height(float(distance), target.camera_height)
                launch_angle = target.find_launch_angle(float(distance), float(target_height), initv, float(wind_spd), float(wind_dir))
                target_angle = launch_angle[0]
                yaw_correction = launch_angle[1]
                over_limit = launch_angle[2]
                if over_limit:
                    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, "Target angle over HW limits".encode())
                    target.set_servo_pwm(target.yaw_channel, 1500)
                    target.set_elev_servo_pwm(target.elev_channel, 1500, pitch)
                    break
                target_yaw = float(yaw) + float(yaw_correction)
                #print(pitch, target_angle, yaw_correction, over_limit, target_height)
                yaw_pwm = yaw_controller.calculate_pwm_signal(yaw, target_yaw, delta_time)
                elevation_pwm = elevation_controller.calculate_pwm_signal(pitch, target_angle, delta_time)
                #print(yaw_pwm, elevation_pwm)
                #print(elevation_controller.integral)
                target.set_servo_pwm(target.yaw_channel, yaw_pwm)
                target.set_elev_servo_pwm(target.elev_channel, elevation_pwm, pitch)
                if float(pitch) == float(target_angle) and 1480 < yaw_pwm < 1520 and 1480 < elevation_pwm < 1520:
                    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, "AIMING DONE!".encode())
                    target.set_servo_pwm(target.yaw_channel, 1500)
                    time.sleep(1)
                    target.set_elev_servo_pwm(target.elev_channel, 1500, pitch)
                    target_attitude_reached_flag = True
                    if air_pressure >= 0:
                        time.sleep(1)
                        target.arm()
                        start_aiming_flag = False
                        target.arm()
                        ready_to_fire_flag = True
                        target.set_servo_pwm(target.start_aiming_channel, 1100)
                        target.set_servo_pwm(target.trigger_channel, 1100)
                        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, "READY TO FIRE!".encode())
                        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, "READY TO FIRE!".encode())
                        #print("ready to fire")
                    else:
                        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, "Not pressurised!".encode())
                        master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, "Not pressurised!".encode())
            
            elif start_aiming_command > 1800 and fire_charge_input > 1800 and ready_to_fire_flag:
                fire_charge_flag = True
               
            elif target_attitude_reached_flag and ready_to_fire_flag and fire_charge_flag:
                master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_ERROR, "FIRING!".encode())
                target.set_servo_pwm(target.trigger_first_channel, 1100) #trigger actuator in
                target.set_servo_pwm(target.trigger_second_channel, 1100) #trigger actuator in
                time.sleep(target.triggering_time)
                target.set_servo_pwm(target.trigger_first_channel, 1500) #trigger actuator out
                target.set_servo_pwm(target.trigger_second_channel, 1500) #trigger actuator out
                time.sleep(target.triggering_time + 1)
                target.set_servo_pwm(target.start_aiming_channel, 1100)
                target.set_servo_pwm(target.trigger_channel, 1100)
                target.disarm()
                ready_to_fire_flag = False
                start_aiming_flag = False
                fire_charge_flag = False
                print("fired")
                target.disarm()
                
            elif start_aiming_command > 1800 and not ready_to_fire_flag:
                start_aiming_flag = True
                target_attitude_reached_flag = False
                
            else:
                servo_yaw_out = yaw_in
                pitch_compensation_out = pitch_in
                target.previous_yaw_pwm = target.update_servo_pwm(target.yaw_channel, servo_yaw_out, target.previous_yaw_pwm)
                target.previous_elev_pwm = target.update_elev_servo_pwm(target.elev_channel, pitch_compensation_out,pitch, target.previous_elev_pwm)
            
            #clamp manual control to min/max as well
            #create a method for shutting down the computer
            #make automatic startup
                        
    except Exception as e:
        target.set_servo_pwm(target.yaw_channel, 1500)
        target.set_elev_servo_pwm(target.elev_channel, 1500, pitch)
        target.disarm()
        target.set_mode(end_mode)
        print(f"Error: {e}", file = sys.stderr)
        sys.exit(1)
        
if __name__ == "__main__":
    main()
