import matplotlib.pyplot as plt

from params import *
from PM_DC import PMDCMotor
from PID import PID
from H_bridge import HBridge_Unipolar

# dynamic variables
cabin_load = 250

#calculate current control (PID) gains. INPUT-> Current error, OUTPUT -> Terminal Voltage Reference
alfa_current_control = math.log(9)/PARAMD_DC_MOTOR_CURRENT_CONTROL_RISE_TIME
current_control_kp = alfa_current_control*PARAM_DC_MOTOR_INDUCTANCE
current_control_ki = alfa_current_control*PARAM_DC_MOTOR_ARMATURE_RESISTANCE
current_control_kd = 0.0
current_control_PID = PID(kp=current_control_kp, ki=current_control_ki, kd=current_control_kd, output_bounds=(-PARAM_DC_MOTOR_V_RATED, PARAM_DC_MOTOR_V_RATED), integral_bounds=(-PARAM_DC_MOTOR_V_RATED/current_control_ki, PARAM_DC_MOTOR_V_RATED/current_control_ki)) # PID controller for current, measures current, sets voltage

print(f"Current Control alfa:{alfa_current_control}, Kp: {current_control_kp}, Ki: {current_control_ki}, Kd: {current_control_kd}")

# Calculate the speed control (PID) gains. INPUT-> Speed error, OUTPUT -> Current Reference
w_to_V_cabin_coefficient = (PARAM_GEAR_SPEED_RATIO*PARAM_DRUM_DIAMETER/4)
cabin_and_cw_reflected_inertia = math.pow(w_to_V_cabin_coefficient,2)*(PARAM_CABIN_EMPTY_MASS+PARAM_CABIN_MAX_LOAD_MASS/2+PARAM_CW_MASS)   # reflected inertia of the CW and Cabin where half load is assumed

alfa_speed_control = math.log(9)/PARAMD_DC_MOTOR_SPEED_CONTROL_RISE_TIME
speed_control_kp = alfa_speed_control*(cabin_and_cw_reflected_inertia+PARAM_DC_MOTOR_AND_GEARS_J)/PARAM_DC_MOTOR_K_T
speed_control_ki = alfa_speed_control*PARAM_DC_MOTOR_TOTAL_VISCOUS_FRICTION/PARAM_DC_MOTOR_K_T
speed_control_kd = 0.0
speed_control_PID = PID(kp=speed_control_kp, ki=speed_control_ki, kd=0.0, output_bounds=(-PARAM_DC_MOTOR_I_RATED, PARAM_DC_MOTOR_I_RATED)) # PID controller for speed, measures speed, sets current

H_bridge_obj = HBridge_Unipolar(Vin_DC=PARAM_RECTIFIER_V_BUS, carrier_magnitude=1, carrier_frequency=7500, v_ref=0)
motor_obj = PMDCMotor(V_rated=PARAM_DC_MOTOR_V_RATED, I_rated=PARAM_DC_MOTOR_I_RATED, R_armature=PARAM_DC_MOTOR_ARMATURE_RESISTANCE, L_armature=PARAM_DC_MOTOR_INDUCTANCE, K_t=PARAM_DC_MOTOR_K_T, K_b=PARAM_DC_MOTOR_K_B,  J_rotor=PARAM_DC_MOTOR_AND_GEARS_J, B_rotor=PARAM_DC_MOTOR_TOTAL_VISCOUS_FRICTION)

time= []
rotor_angular_speed = []
rotor_desired_angular_speed = []
armature_current = []
current_reference = []
V_cabin_desired = 0
counter = 0

t = 0
while t < 15:

    if t < 2:
        V_cabin_desired = t/1
    elif t>4 and t<6:
        V_cabin_desired = 2 - (t-4)/1
    elif t>8 and t<10:
        V_cabin_desired = -(t-8)
    elif t>12 and t<14:
        V_cabin_desired = -2+(t-12)
   
    w_desired = V_cabin_desired*50

    cabin_total_mass = PARAM_CABIN_EMPTY_MASS + cabin_load    
    T_load_referred = 0.196* (cabin_total_mass - PARAM_CW_MASS)
    
    current_to_apply = speed_control_PID.update(setpoint=w_desired, process_variable=motor_obj.get_speed_rad_s(), dt = PARAM_DT)
    voltage_to_apply = current_control_PID.update(setpoint=current_to_apply, process_variable=motor_obj.get_current(), dt = PARAM_DT)
    H_bridge_obj.update_vref_for_desired_vout(t=t, desired_voltage_output=voltage_to_apply)
    voltage_to_apply = H_bridge_obj.calculate_Vout(t=t)

    motor_obj.update_armature_current(terminal_voltage=voltage_to_apply, dt=PARAM_DT)
    motor_obj.update_rotor_angular_velocity(dt=PARAM_DT, external_refered_load_torque=T_load_referred, external_refered_viscous_friction=PARAM_DC_MOTOR_TOTAL_VISCOUS_FRICTION, external_refered_inertia=cabin_and_cw_reflected_inertia)
    
    if counter % 50 == 0:
        time.append(t)
        rotor_angular_speed.append(motor_obj.get_speed_rad_s())
        rotor_desired_angular_speed.append(w_desired)
        armature_current.append(motor_obj.get_current())
        current_reference.append(current_to_apply)

    t += PARAM_DT
    counter += 1

plt.figure()
plt.plot(time, rotor_angular_speed, time, rotor_desired_angular_speed)
plt.xlabel('Time (s)')
plt.ylabel('Rotor Angular Speed (Rad/s)')
plt.title('Rotor Speed vs Time')
plt.grid()

plt.figure()
plt.plot(time, armature_current, label='Armature Current')
plt.plot(time, current_reference, label='Current Reference')
plt.xlabel('Time (s)')
plt.ylabel('Armature Current (A)')
plt.title('Armature Current vs Time')
plt.grid()
plt.legend()



plt.show()


