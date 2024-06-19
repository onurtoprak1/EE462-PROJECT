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

    motor_obj.update_armature_current(terminal_voltage=voltage_to_apply, dt=PARAM_DT)
    motor_obj.update_rotor_angular_velocity(dt=PARAM_DT, external_refered_load_torque=load_torque, external_refered_viscous_friction=0, external_refered_inertia=0)
    
    t += PARAM_DT

    if counter % 1000 == 0 and False:
        pass
        print(f"Time: {t:.2f} s, Speed: {motor_obj.get_speed_rad_s()} Current: {motor_obj.get_current():.2f} A, Applied Voltage: {voltage_to_apply:.2f} V")
    
    time.append(t)
    speed.append(motor_obj.get_speed_rad_s())
    current.append(motor_obj.get_current())
    test.append(voltage_to_apply)
    h_bridge_voltage.append(H_bridge_obj.calculate_Vout(t=t))
    
    counter += 1

plt.figure()
plt.plot(time, speed)
plt.xlabel('Time (s)')
plt.ylabel('Speed (rad/s)')
plt.title('Motor Speed vs Time')

plt.figure()
plt.plot(time, current)
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('Motor Current vs Time')

plt.figure()
plt.plot(time, test)
plt.xlabel('Time (s)')
plt.ylabel('Tested Value (*)')
plt.title('Tested Value vs Time')

plt.figure()
plt.plot(time, h_bridge_voltage)
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('H-Bridge Output Voltage vs Time')


plt.show()
