class Servo_motor:
    def __init__(self, PWM_FREQ):
        self.pwm_freq = PWM_FREQ

    def servo1_angle_to_duty_cycle(self, servo1_angle):
        servo1_duty_cycle = (0.05 * self.pwm_freq) + (0.19 * self.pwm_freq * servo1_angle / 180)
        return servo1_duty_cycle

    def servo2_angle_to_duty_cycle(self, servo1_angle=0):
        servo2_duty_cycle = (0.05 * self.pwm_freq) + (0.19 * self.pwm_freq * servo1_angle / 180)
        return servo2_duty_cycle

    def servo3_angle_to_duty_cycle(self, servo1_angle=0):
        servo3_duty_cycle = (0.05 * self.pwm_freq) + (0.19 * self.pwm_freq * servo1_angle / 180)
        return servo3_duty_cycle

    def servo4_angle_to_duty_cycle(self, servo1_angle=0):
        servo4_duty_cycle = (0.05 * self.pwm_freq) + (0.19 * self.pwm_freq * servo1_angle / 180)
        return servo4_duty_cycle


# def servo1_angle_to_duty_cycle(servo1_angle):
#     servo1_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
#     return servo1_duty_cycle
#
#
# def servo2_angle_to_duty_cycle(servo1_angle=0):
#     servo2_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
#     return servo2_duty_cycle
#
#
# def servo3_angle_to_duty_cycle(servo1_angle=0):
#     servo3_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
#     return servo3_duty_cycle
#
#
# def servo4_angle_to_duty_cycle(servo1_angle=0):
#     servo4_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
#     return servo4_duty_cycle
