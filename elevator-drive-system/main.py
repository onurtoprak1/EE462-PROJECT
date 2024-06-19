from PM_DC import PMDCMotor
from PID import PID


from params import *

motor_obj = PMDCMotor(V_rated=PARAM_DC_MOTOR_V_RATED, I_rated=PARAM_DC_MOTOR_I_RATED, R_armature=PARAM_DC_MOTOR_ARMATURE_RESISTANCE, L_armature=PARAM_DC_MOTOR_INDUCTANCE, K_t=PARAM_DC_MOTOR_K_T, K_b=PARAM_DC_MOTOR_K_B,  J_rotor=PARAM_DC_MOTOR_AND_GEARS_J, B_rotor=PARAM_DC_MOTOR_VISCOUS_FRICTION)
current_control_PID = PID(kp=PARAM_DC_MOTOR_CURRENT_CONTROL_Kp, ki=PARAM_DC_MOTOR_CURRENT_CONTROL_Ki, kd=0.0, output_bounds=(-PARAM_DC_MOTOR_V_RATED, PARAM_DC_MOTOR_V_RATED) ) # PID controller for current, measures current, sets voltage

t = 0
dt = 1e-5

counter = 0
while t < 10:
    setpoint = 50 # A
    current = motor_obj.get_current()

    voltage_to_apply = current_control_PID.update(setpoint=setpoint, process_variable=current, dt = dt)
    
    t += dt

    if counter %1000 == 0:
        print(f"Time: {t:.2f} s, Current: {current:.2f} A, Applied Voltage: {voltage_to_apply:.2f} V")
    counter += 1