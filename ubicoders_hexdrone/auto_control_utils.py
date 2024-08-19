
class MyPID:
    def __init__(self, kp=1, ki=1, kd=0, sp = 0, int_limit=0.2, output_limit=0.5) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit =int_limit
        self.error = 0
        self.sp = 0
        self.error_int = 0
        self.output_limit = output_limit    
    
    def update_sp(self, sp):
        self.sp = sp

    def update_error_int(self):
        self.error_int += self.error
        if self.error_int > self.limit:
            self.error_int = self.limit
        elif self.error_int < -self.limit:
            self.error_int = -self.limit
    
    def update_current_value(self, current_value):
        self.current_value = current_value
        self.error = self.sp - self.current_value
        self.error_prev = self.error
        self.update_error_int()
        return self.calc_output()

    def calc_output(self):
        output = self.kp * self.error + self.ki * self.error_int + self.kd * (self.error - self.error_prev)    
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
        return output

class AutoPosControl:
    def __init__(self) -> None:
        self.current_throttle = 0
        self.current_px = 0
        self.current_py = 0
        
        self.px_sp = 0
        self.py_sp = 0
        self.pz_sp = -0.5

        self.height_controller = MyPID(kp=1, ki=1, sp=self.pz_sp, int_limit=0.1, output_limit=0.7)

    def gen_throttle_sp(self, throttle_sp):
        # new_throttle_sp < self.current_throttle + 0.1
        throttle_inc = 0.1
        if throttle_sp > self.current_throttle + throttle_inc:
            throttle_sp = self.current_throttle + throttle_inc
        elif throttle_sp < self.current_throttle - throttle_inc:
            throttle_sp = self.current_throttle -throttle_inc
        
        if throttle_sp > self.throttle_max:
            throttle_sp = self.throttle_max
      


    def update_throttle(self, throttle):
        print(f"current throttle: {throttle}")
        self.height_controller.sp = self.pz_sp
        throttle_sp = self.height_controller.update_current_value(throttle)        
        self.throttle_sp = self.gen_throttle_sp(throttle_sp)

