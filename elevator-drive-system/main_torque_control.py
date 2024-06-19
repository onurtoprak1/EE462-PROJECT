import matplotlib.pyplot as plt

from params import *
from PM_DC import PMDCMotor
from PID import PID
from H_bridge import HBridge_Unipolar

H_bridge_obj = HBridge_Unipolar(Vin_DC=PARAM_RECTIFIER_V_BUS, carrier_magnitude=1, carrier_frequency=2500, v_ref=0)
motor_obj = PMDCMotor(V_rated=PARAM_DC_MOTOR_V_RATED, I_rated=PARAM_DC_MOTOR_I_RATED, R_armature=PARAM_DC_MOTOR_ARMATURE_RESISTANCE, L_armature=PARAM_DC_MOTOR_INDUCTANCE, K_t=PARAM_DC_MOTOR_K_T, K_b=PARAM_DC_MOTOR_K_B,  J_rotor=PARAM_DC_MOTOR_AND_GEARS_J, B_rotor=PARAM_DC_MOTOR_VISCOUS_FRICTION)
current_control_PI = PID(kp=PARAM_DC_MOTOR_CURRENT_CONTROL_Kp, ki=PARAM_DC_MOTOR_CURRENT_CONTROL_Ki, kd=0.0, output_bounds=PARAM_DC_MOTOR_CURRENT_CONTROL_OUTPUT_BOUNDS ) # PID controller for current, measures current, sets voltage
speed_control_PI = PID(kp=PARAM_DC_MOTOR_SPEED_CONTROL_Kp, ki=PARAM_DC_MOTOR_SPEED_CONTROL_Ki, kd=0.0, output_bounds=PARAM_DC_MOTOR_SPEED_CONTROL_OUTPUT_BOUNDS) # PID controller for speed, measures speed, sets current

t = 0
counter = 0

time = []
speed = []
current = []
test = []
h_bridge_voltage = []

load_torque = 125
while t < 1:
    current_to_apply = speed_control_PI.update(setpoint=PARAM_DC_MOTOR_RATED_RAD_S/2, process_variable=motor_obj.get_speed_rad_s(), dt = PARAM_DT)
    current_to_apply +=load_torque/PARAM_DC_MOTOR_K_T #Feedforward control 
    voltage_to_apply = current_control_PI.update(setpoint=current_to_apply, process_variable=motor_obj.get_current(), dt = PARAM_DT)
    H_bridge_obj.update_vref_for_desired_vout(t=t, desired_voltage_output=voltage_to_apply)
    H_bridge_output_voltage = H_bridge_obj.calculate_Vout(t=t)

    motor_obj.update_armature_current(terminal_voltage=H_bridge_output_voltage, dt=PARAM_DT)
    motor_obj.update_rotor_angular_velocity(dt=PARAM_DT, external_refered_load_torque=load_torque, external_refered_viscous_friction=0, external_refered_inertia=dc_motor_reflected_inertia)
    
    t += PARAM_DT

