
class PMDCMotor:

    def __init__(self, V_rated:float = None, I_rated:float = None, R_armature:float = None, L_armature:float = None, K_b:float = None, K_t:float = None, J_rotor:float = None, B_rotor:float = 0) -> None:
        self.V_RATED = V_rated
        self.I_RATED = I_rated
        self.R_ARMATURE = R_armature
        self.L_ARMATURE = L_armature
        self.J_ROTOR = J_rotor
        self.B_ROTOR = B_rotor
        self.K_B = K_b
        self.K_T = K_t

        self.current = 0
        self.angular_velocity = 0

    def get_speed_rad_s(self)->float:
        return self.angular_velocity
    
    def get_speed_rpm(self)->float:
        return (self.angular_velocity/ (2 * 3.14159)) * 60
    
    def get_current(self)->float:
        return self.current
    
    def update_armature_current(self, terminal_voltage:float = None, dt:float = None) ->float:
        v_resistor = self.current * self.R_ARMATURE
        v_back_emf = self.K_B * self.angular_velocity
        v_inductor = terminal_voltage - v_resistor - v_back_emf

        self.current += (v_inductor / self.L_ARMATURE) * dt
        return self.current
    
    def update_rotor_angular_velocity(self, dt:float=None, external_refered_load_torque:float = 0, external_refered_viscous_friction:float = 0, external_refered_inertia:float=0) -> float:
        T_em = self.K_T * self.current

        net_torque = T_em - external_refered_load_torque - (self.B_ROTOR + external_refered_viscous_friction )* self.angular_velocity
        net_acceleration = net_torque / (self.J_ROTOR+external_refered_inertia)

        self.angular_velocity += net_acceleration*dt

    


