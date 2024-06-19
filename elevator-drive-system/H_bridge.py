

class HBridge_Unipolar:

    def __init__(self, Vin_DC:float = None, carrier_magnitude:float = 1, carrier_frequency:float = 10000, v_ref:float = 0 ) -> None:
        self.VIN_DC = Vin_DC
        self.CARRIER_FREQUENCY = carrier_frequency
        self.CARRIER_PERIOD = 1/self.CARRIER_FREQUENCY
        self.CARRIER_MAGNITUDE = carrier_magnitude
        self.v_ref = v_ref

    def carrier_signal_value(self, t: float = None) -> float:
        mod_t = t % self.CARRIER_PERIOD
        half_period = self.CARRIER_PERIOD / 2
        double_magnitude = 2 * self.CARRIER_MAGNITUDE

        if mod_t < half_period:
            return -self.CARRIER_MAGNITUDE + (mod_t / half_period) * double_magnitude
        else:
            mod_t -= half_period
            return self.CARRIER_MAGNITUDE - (mod_t / half_period) * double_magnitude


    def update_vref_for_desired_vout(self, desired_voltage_output:float = None, t:float=None)->float:
        if self.carrier_signal_value(t) > -0.80*self.CARRIER_MAGNITUDE: #To ensure the vref is not updated at the wrong time
            return self.v_ref
        self.v_ref = min(self.CARRIER_MAGNITUDE, max(-self.CARRIER_MAGNITUDE, desired_voltage_output/self.VIN_DC))

       

    def calculate_Vout(self, t:float = None) -> float:
        carrier_value = self.carrier_signal_value(t)     
        Van = self.VIN_DC if carrier_value < self.v_ref else 0
        Vbn = self.VIN_DC if carrier_value < -self.v_ref else 0
        Vab = Van - Vbn
        return Vab

    

