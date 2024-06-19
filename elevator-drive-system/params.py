import math

PARAM_DT = 10e-6 # s

PARAM_RECTIFIER_V_BUS = 200.0 # V

PARAM_G = 9.81 # m/s^2
PARAM_DRUM_DIAMETER= 0.4 # m
PARAM_CW_MASS = 1250 # kg
PARAM_CABIN_EMPTY_MASS = 1000 # kg
PARAM_CABIN_MAX_LOAD_MASS = 500 # kg
PARAM_GEAR_SPEED_RATIO = 1/5 # w_drum / w_motor

PARAM_DC_MOTOR_RATED_RAD_S= 115 # rad/s
PARAM_DC_MOTOR_V_RATED = 200 # V
PARAM_DC_MOTOR_I_RATED = 75 # A
PARAM_DC_MOTOR_ARMATURE_RESISTANCE = 0.1 # Ohms
PARAM_DC_MOTOR_INDUCTANCE = 0.002 # H
PARAM_DC_MOTOR_K_T = 1.75 # Nm/A
PARAM_DC_MOTOR_K_B = 1.75 # V/(rad/s)
PARAM_DC_MOTOR_AND_GEARS_J = 0.2 # kg*m^2
PARAM_DC_MOTOR_TOTAL_VISCOUS_FRICTION = 0.0 # Nm/(rad/s)
PARAM_DC_MOTOR_RATED_TORQUE = PARAM_DC_MOTOR_I_RATED * PARAM_DC_MOTOR_K_T # Nm


PARAMD_DC_MOTOR_CURRENT_CONTROL_RISE_TIME = 0.010 # s
PARAMD_DC_MOTOR_SPEED_CONTROL_RISE_TIME = 7.5*PARAMD_DC_MOTOR_CURRENT_CONTROL_RISE_TIME
