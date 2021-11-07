import cv2
import time
import numpy as np
import RPi.GPIO as GPIO
from DC_Motor import DC_motor
from Servo_Motor import Servo_motor
from pyPS4Controller.controller import Controller


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# DC
motor = DC_motor(11, 13, 15, 33, 37, 35, 36, 38, 40, 8, 12, 10)
motor_speed = 100
motor.speed(motor_speed)

# Servo
PWM_FREQ = 50
servo_motor = Servo_motor(PWM_FREQ)
servo1 = 16
servo2 = 18
servo3 = 31
servo4 = 32
servo1_angle = 0
servo2_angle = 50
servo3_angle = 0
servo4_angle = 90

GPIO.setup(servo1, GPIO.OUT)
GPIO.setup(servo2, GPIO.OUT)
GPIO.setup(servo3, GPIO.OUT)
GPIO.setup(servo4, GPIO.OUT)

pwm1 = GPIO.PWM(servo1, PWM_FREQ)
pwm1.start(0)
pwm2 = GPIO.PWM(servo2, PWM_FREQ)
pwm2.start(0)
pwm3 = GPIO.PWM(servo3, PWM_FREQ)
pwm3.start(0)
pwm4 = GPIO.PWM(servo4, PWM_FREQ)
pwm4.start(0)

# Auto_left
i = 1


# ------------------------------PS4_Controller------------------------------------
# create a custom class for your controller and subclass Controller
# each def function is for each button
class MyController(Controller):
    def on_x_press(self):
        print('turn right')
        motor.turn_right()

    def on_x_release(self):
        motor.stop()

    def on_triangle_press(self):
        print('turn left')
        motor.turn_left()

    def on_triangle_release(self):
        motor.stop()

    def on_circle_press(self):
        print('forward')
        motor.forward()

    def on_circle_release(self):
        motor.stop()

    def on_square_press(self):
        print("back")
        motor.backward()

    def on_square_release(self):
        motor.stop()

    def on_L1_press(self):
        global servo1_angle
        if servo1_angle < 180:
            servo1_angle += 5
            if servo1_angle > 180:
                servo1_angle = 180
            dc1 = servo_motor.servo1_angle_to_duty_cycle(servo1_angle)
            pwm1.ChangeDutyCycle(dc1)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo1_angle, dc1))
            time.sleep(0.2)

    def on_L1_release(self):
        global servo1_angle
        pwm1.ChangeDutyCycle(servo_motor.servo1_angle_to_duty_cycle(servo1_angle))

    def on_L2_press(self, value):
        global servo1_angle
        if value > 32760:
            if servo1_angle > 0:
                servo1_angle -= 5
                if servo1_angle < 0:
                    servo1_angle = 0
                dc1 = servo_motor.servo1_angle_to_duty_cycle(servo1_angle)
                pwm1.ChangeDutyCycle(dc1)
                print('角度={: >3}, 工作週期={:.2f}'.format(servo1_angle, dc1))
                time.sleep(0.2)

    def on_L2_release(self):
        global servo1_angle
        pwm1.ChangeDutyCycle(servo_motor.servo1_angle_to_duty_cycle(servo1_angle))

    def on_R1_press(self):
        global servo2_angle
        if servo2_angle < 180:
            servo2_angle += 5
            if servo2_angle > 180:
                servo2_angle = 180
            dc2 = servo_motor.servo2_angle_to_duty_cycle(servo2_angle)
            pwm2.ChangeDutyCycle(dc2)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo2_angle, dc2))
            time.sleep(0.2)

    def on_R1_release(self):
        global servo2_angle
        pwm2.ChangeDutyCycle(servo_motor.servo2_angle_to_duty_cycle(servo2_angle))

    def on_R2_press(self, value):
        global servo2_angle
        if value > 32760:
            if servo2_angle > 0:
                servo2_angle -= 5
                if servo2_angle < 0:
                    servo2_angle = 0
                dc2 = servo_motor.servo2_angle_to_duty_cycle(servo2_angle)
                pwm2.ChangeDutyCycle(dc2)
                print('角度={: >3}, 工作週期={:.2f}'.format(servo2_angle, dc2))
                time.sleep(0.2)

    def on_R2_release(self):
        global servo2_angle
        pwm2.ChangeDutyCycle(servo_motor.servo2_angle_to_duty_cycle(servo2_angle))

    def on_up_arrow_press(self):
        global servo3_angle
        if servo3_angle < 180:
            servo3_angle += 5
            if servo3_angle > 180:
                servo3_angle = 180
            dc3 = servo_motor.servo3_angle_to_duty_cycle(servo3_angle)
            pwm3.ChangeDutyCycle(dc3)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo3_angle, dc3))
            time.sleep(0.2)

    def on_up_down_arrow_release(self):
        global servo3_angle
        pwm3.ChangeDutyCycle(servo_motor.servo3_angle_to_duty_cycle(servo3_angle))

    def on_down_arrow_press(self):
        global servo3_angle
        if servo3_angle > 0:
            servo3_angle -= 5
            if servo3_angle < 0:
                servo3_angle = 0
            dc3 = servo_motor.servo3_angle_to_duty_cycle(servo3_angle)
            pwm3.ChangeDutyCycle(dc3)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo3_angle, dc3))
            time.sleep(0.2)

    def on_left_arrow_press(self):
        global servo4_angle
        if servo4_angle > 0:
            servo4_angle -= 5
            if servo4_angle < 1:
                servo4_angle = 1
            dc4 = servo_motor.servo4_angle_to_duty_cycle(servo4_angle)
            pwm4.ChangeDutyCycle(dc4)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo4_angle, dc4))
            time.sleep(0.2)

    def on_left_right_arrow_release(self):
        global servo4_angle
        pwm4.ChangeDutyCycle(servo_motor.servo4_angle_to_duty_cycle(servo4_angle))

    def on_right_arrow_press(self):
        global servo4_angle
        if servo4_angle < 180:
            servo4_angle += 5
            if servo4_angle > 180:
                servo4_angle = 180
            dc4 = servo_motor.servo4_angle_to_duty_cycle(servo4_angle)
            pwm4.ChangeDutyCycle(dc4)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo4_angle, dc4))
            time.sleep(0.2)

    def on_L3_up(self, value):
        print("on_L3_up: {}".format(value))

    def on_L3_down(self, value):
        print("on_L3_down: {}".format(value))

    def on_L3_left(self, value):
        print("on_L3_left: {}".format(value))

    def on_L3_right(self, value):
        print("on_L3_right: {}".format(value))

    def on_L3_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        print("on_L3_at_rest")

    def on_L3_press(self):
        """L3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        print("on_L3_press")

    def on_L3_release(self):
        """L3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        print("on_L3_release")

    def on_R3_up(self, value):
        print("on_R3_up: {}".format(value))

    def on_R3_down(self, value):
        print("on_R3_down: {}".format(value))

    def on_R3_left(self, value):
        print("on_R3_left: {}".format(value))

    def on_R3_right(self, value):
        print("on_R3_right: {}".format(value))

    def on_R3_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        print("on_R3_at_rest")

    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        print("on_R3_press")

    def on_R3_release(self):
        """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        print("on_R3_release")

    def on_options_press(self):
        print("Auto start")
        global i
        while i == 1:
            if i == 1:
                motor.forward()
                print('forward')
                time.sleep(0.9)
                motor.stop()
                i = 2

            if i == 2:
                cap = cv2.VideoCapture(0)
                print('Detect')
                while True:
                    # Capture Video from Camera
                    _, frame = cap.read()

                    frame = cv2.GaussianBlur(frame, (11, 11), 0)
                    # Convert BGR to HSV
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    # define range of red color in HSV
                    lower_red = np.array([-10, 100, 100])
                    upper_red = np.array([10, 255, 255])

                    # define range of green color in HSV
                    lower_green = np.array([50, 100, 100])
                    upper_green = np.array([70, 255, 255])

                    # Threshold the HSV image to get only green colors
                    mask_green = cv2.inRange(hsv, lower_green, upper_green)
                    # Threshold the HSV image to get only red colors
                    mask_red = cv2.inRange(hsv, lower_red, upper_red)
                    # Bitwise-AND mask and original image

                    res_red = cv2.bitwise_and(frame, frame, mask=mask_red)
                    res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

                    cv2.imshow('mask_green', mask_green)
                    cv2.waitKey(1)

                    (contours_red, _) = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    (contours_green, _) = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours_green) > 0:
                        print('forward')
                        motor.forward()
                        time.sleep(0.25)
                        print('green light')
                        i = 3
                        cv2.destroyAllWindows()
                        break

            if i == 3:
                cap.release()
                print('turn right')
                motor.turn_right()
                time.sleep(0.19)

                print('forward')
                motor.forward()
                time.sleep(1.25)

                print('turn left')
                motor.turn_left()
                time.sleep(0.45)

                print('forward')
                motor.forward()
                time.sleep(1)

                print('turn right')
                motor.turn_right()
                time.sleep(0.5)
                motor.forward()
                time.sleep(0.5)
                motor.stop()
                i = 4

            if i == 4:
                print('Detect')
                cap = cv2.VideoCapture(0)
                while True:
                    # Capture Video from Camera
                    _, frame = cap.read()

                    frame = cv2.GaussianBlur(frame, (11, 11), 0)
                    # Convert BGR to HSV
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    # define range of red color in HSV
                    lower_red = np.array([-10, 100, 100])
                    upper_red = np.array([10, 255, 255])

                    # define range of green color in HSV
                    lower_green = np.array([50, 90, 100])
                    upper_green = np.array([70, 255, 255])

                    # Threshold the HSV image to get only green colors
                    mask_green = cv2.inRange(hsv, lower_green, upper_green)
                    # Threshold the HSV image to get only red colors
                    mask_red = cv2.inRange(hsv, lower_red, upper_red)
                    # Bitwise-AND mask and original image

                    res_red = cv2.bitwise_and(frame, frame, mask=mask_red)
                    res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

                    cv2.imshow('mask_green', mask_green)
                    cv2.waitKey(1)

                    (contours_red, _) = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    (contours_green, _) = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours_green) > 0:
                        print('green light')
                        i = 5
                        cv2.destroyAllWindows()
                        break

            if i == 5:
                cap.release()
                print('forward')
                motor.forward()
                time.sleep(0.65)
                print('final')
                motor.stop()
                i = 6

            elif i == 6:
                print("forward")
                time.sleep(0.5)
                print("stop")
                i = 0
                break
        while i == 0:
            break

    def on_options_release(self):
        print("on_options_release")

    def on_share_press(self):
        print("Door close")
        motor.door_close()

    def on_share_release(self):
        motor.door_stop()

    def on_playstation_button_press(self):
        print("Door open")
        motor.door_open()

    def on_playstation_button_release(self):
        motor.door_stop()


# now make sure the controller is paired over the Bluetooth and turn on the listener
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)
