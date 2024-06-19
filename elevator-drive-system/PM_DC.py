
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

    def get_speed_rpm(self):
        return (self.angular_velocity/ (2 * 3.14159)) * 60
    
    def get_current(self):
        return self.current
    
    


