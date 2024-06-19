import math

PARAM_DC_MOTOR_V_RATED = 200 # V
PARAM_DC_MOTOR_I_RATED = 75 # A
PARAM_DC_MOTOR_ARMATURE_RESISTANCE = 0.1 # Ohms
PARAM_DC_MOTOR_INDUCTANCE = 0.002 # H
PARAM_DC_MOTOR_K_T = 1.75 # Nm/A
PARAM_DC_MOTOR_K_B = 1.75 # V/(rad/s)
PARAM_DC_MOTOR_AND_GEARS_J = 0.2 # kg*m^2
PARAM_DC_MOTOR_VISCOUS_FRICTION = 0.0 # Nm/(rad/s)


PARAM_DC_MOTOR_CURRENT_CONTROL_RISE_TIME = 0.010 # s
alfa_current_control = math.log(9)/PARAM_DC_MOTOR_CURRENT_CONTROL_RISE_TIME
PARAM_DC_MOTOR_CURRENT_CONTROL_Kp = alfa_current_control*PARAM_DC_MOTOR_INDUCTANCE
PARAM_DC_MOTOR_CURRENT_CONTROL_Ki = alfa_current_control*PARAM_DC_MOTOR_ARMATURE_RESISTANCE

print(PARAM_DC_MOTOR_CURRENT_CONTROL_Kp, PARAM_DC_MOTOR_CURRENT_CONTROL_Ki)