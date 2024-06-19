

class PID:
    def __init__(self, kp:float=1, ki:float=0, kd:float=0, output_bounds:tuple=(float('-inf'), float('inf')), integral_bounds:tuple=(float('-inf'), float('inf'))):
        self.OUTPUT_BOUNDS = output_bounds
        self.INTEGRAL_BOUNDS = integral_bounds
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.prev_error = 0
        self.integral = 0
        self.output = None

    def clamp(self, value:float, bounds:tuple):
        return max(bounds[0], min(bounds[1], value))
    
    def get_integral(self):
        return self.integral
    
    def get_error(self):
        return self.prev_error
    
    def update(self, setpoint:float=None, process_variable:float=None, dt:float=None) -> float:
        error = setpoint - process_variable

        self.integral += error * dt
        self.integral = self.clamp(self.integral, self.INTEGRAL_BOUNDS) # Prevent integral windup

        derivative = (error - self.prev_error) / dt

        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.output = self.clamp(self.output, self.OUTPUT_BOUNDS)

        self.prev_error = error

        return self.output

       

    