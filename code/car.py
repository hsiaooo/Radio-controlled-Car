from pyPS4Controller.controller import Controller
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np

#--------------------------------DC_Motor----------------------------------------
class DC_Motor:
    def __init__(self, ena, in1, in2, enb, in3, in4, enc, in5, in6, end, in7, in8, ene, in9, in10, enf, in11, in12):
        self.ENA = ena
        self.IN1 = in1
        self.IN2 = in2
        self.ENB = enb
        self.IN3 = in3
        self.IN4 = in4
        self.ENC = enc
        self.IN5 = in5
        self.IN6 = in6
        self.END = end
        self.IN7 = in7
        self.IN8 = in8
        self.ENE = ene
        self.IN9 = in9
        self.IN10 = in10
        self.ENF = enf
        self.IN11 = in11
        self.IN12 = in12
        # 設定後右輪
        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.LOW)
        # 利用PWN來改變車子馬達轉速（改變車子速度），初始值為全速100
        self.ENA_SPEED = GPIO.PWM(self.ENA, 600)
        self.ENA_SPEED.start(0)
        self.ENA_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)
        # 設定前右輪
        GPIO.setup(self.ENC, GPIO.OUT, initial=GPIO.LOW)
        self.ENC_SPEED = GPIO.PWM(self.ENC, 600)
        self.ENC_SPEED.start(0)
        self.ENC_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN5, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN6, GPIO.OUT, initial=GPIO.LOW)
        # 設定後左輪
        GPIO.setup(self.ENB, GPIO.OUT, initial=GPIO.LOW)
        self.ENB_SPEED = GPIO.PWM(self.ENB, 600)
        self.ENB_SPEED.start(0)
        self.ENB_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN4, GPIO.OUT, initial=GPIO.LOW)
        # 設定前左輪
        GPIO.setup(self.END, GPIO.OUT, initial=GPIO.LOW)
        self.END_SPEED = GPIO.PWM(self.END, 600)
        self.END_SPEED.start(0)
        self.END_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN7, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN8, GPIO.OUT, initial=GPIO.LOW)
        # 設定下面拉門馬達
        GPIO.setup(self.ENE, GPIO.OUT, initial=GPIO.LOW)
        self.ENE_SPEED = GPIO.PWM(self.ENE, 600)
        self.ENE_SPEED.start(0)
        self.ENE_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN9, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN10, GPIO.OUT, initial=GPIO.LOW)
        # 設定上面拉門馬達
        GPIO.setup(self.ENF, GPIO.OUT, initial=GPIO.LOW)
        self.ENF_SPEED = GPIO.PWM(self.ENF, 600)
        self.ENF_SPEED.start(0)
        self.ENF_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN11, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN12, GPIO.OUT, initial=GPIO.LOW)
    #Speed
    def right_speed(self, speed):
        self.ENA_SPEED.ChangeDutyCycle(speed)
        self.END_SPEED.ChangeDutyCycle(speed)

    def left_speed(self, speed):
        self.ENB_SPEED.ChangeDutyCycle(speed)
        self.ENC_SPEED.ChangeDutyCycle(speed)

    def speed(self, speed):
        self.right_speed(speed)
        self.left_speed(speed)

    #Move
    def forward(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, True)
        GPIO.output(self.IN2, False)
        GPIO.output(self.IN3, True)
        GPIO.output(self.IN4, False)
        GPIO.output(self.IN5, True)
        GPIO.output(self.IN6, False)
        GPIO.output(self.IN7, True)
        GPIO.output(self.IN8, False)

    def backward(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, False)
        GPIO.output(self.IN2, True)
        GPIO.output(self.IN3, False)
        GPIO.output(self.IN4, True)
        GPIO.output(self.IN5, False)
        GPIO.output(self.IN6, True)
        GPIO.output(self.IN7, False)
        GPIO.output(self.IN8, True)

    def turn_left(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, True)
        GPIO.output(self.IN2, False)
        GPIO.output(self.IN3, False)
        GPIO.output(self.IN4, True)
        GPIO.output(self.IN5, False) # 先測試看一輪動不動得了
        GPIO.output(self.IN6, True)
        GPIO.output(self.IN7, True)
        GPIO.output(self.IN8, False)

    def turn_right(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, False)
        GPIO.output(self.IN2, True)
        GPIO.output(self.IN3, True)
        GPIO.output(self.IN4, False)
        GPIO.output(self.IN5, True)
        GPIO.output(self.IN6, False)
        GPIO.output(self.IN7, False) # 先測試一輪動不動得了
        GPIO.output(self.IN8, True)

    def stop(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, False)
        GPIO.output(self.IN2, False)
        GPIO.output(self.IN3, False)
        GPIO.output(self.IN4, False)
        GPIO.output(self.IN5, False)
        GPIO.output(self.IN6, False)
        GPIO.output(self.IN7, False)
        GPIO.output(self.IN8, False)

    

#--------------------------------Servo_Mptor------------------------------------------
def servo1_angle_to_duty_cycle(servo1_angle):
    servo1_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
    return servo1_duty_cycle

def servo2_angle_to_duty_cycle(servo1_angle=0):
    servo2_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
    return servo2_duty_cycle

def servo3_angle_to_duty_cycle(servo1_angle=0):
    servo3_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
    return servo3_duty_cycle

def servo4_angle_to_duty_cycle(servo1_angle=0):
    servo4_duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * servo1_angle / 180)
    return servo4_duty_cycle

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

#DC
motor = DC_Motor(11, 13, 15, 33, 37, 35, 36, 38, 40, 8, 12, 10)
motor_speed = 100
motor.speed(motor_speed)

#Servo
PWM_FREQ = 50
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


#Auto_left
i = 1

#------------------------------PS4_Controller------------------------------------
class MyController(Controller):  # create a custom class for your controller and subclass Controller
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
            dc1 = servo1_angle_to_duty_cycle(servo1_angle)
            pwm1.ChangeDutyCycle(dc1)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo1_angle, dc1))
            time.sleep(0.2)
            
            

    def on_L1_release(self):
        global servo1_angle
        pwm1.ChangeDutyCycle(servo1_angle_to_duty_cycle(servo1_angle))

    def on_L2_press(self, value):
        global servo1_angle
        if value > 32760 :
            if servo1_angle > 0:
                servo1_angle -= 5
                if servo1_angle < 0:
                    servo1_angle = 0
                dc1 = servo1_angle_to_duty_cycle(servo1_angle)
                pwm1.ChangeDutyCycle(dc1)
                print('角度={: >3}, 工作週期={:.2f}'.format(servo1_angle, dc1))
                time.sleep(0.2)
                    

    def on_L2_release(self):
        global servo1_angle
        pwm1.ChangeDutyCycle(servo1_angle_to_duty_cycle(servo1_angle))
        

    def on_R1_press(self):
        global servo2_angle
        if servo2_angle < 180:
            servo2_angle += 5
            if servo2_angle > 180:
                servo2_angle = 180
            dc2 = servo2_angle_to_duty_cycle(servo2_angle)
            pwm2.ChangeDutyCycle(dc2)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo2_angle, dc2))
            time.sleep(0.2)
            

    def on_R1_release(self):
        global servo2_angle
        pwm2.ChangeDutyCycle(servo2_angle_to_duty_cycle(servo2_angle))

    def on_R2_press(self, value):
        global servo2_angle
        if value > 32760 :
            if servo2_angle > 0:
                servo2_angle -= 5
                if servo2_angle < 0:
                    servo2_angle = 0
                dc2 = servo2_angle_to_duty_cycle(servo2_angle)
                pwm2.ChangeDutyCycle(dc2)
                print('角度={: >3}, 工作週期={:.2f}'.format(servo2_angle, dc2))
                time.sleep(0.2)
                

    def on_R2_release(self):
        global servo2_angle
        pwm2.ChangeDutyCycle(servo2_angle_to_duty_cycle(servo2_angle))

    def on_up_arrow_press(self):
        global servo3_angle
        if servo3_angle < 180:
            servo3_angle += 5
            if servo3_angle > 180:
                servo3_angle = 180
            dc3 = servo3_angle_to_duty_cycle(servo3_angle)
            pwm3.ChangeDutyCycle(dc3)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo3_angle, dc3))
            time.sleep(0.2)
            

    def on_up_down_arrow_release(self):
        global servo3_angle
        pwm3.ChangeDutyCycle(servo3_angle_to_duty_cycle(servo3_angle))

    def on_down_arrow_press(self):
        global servo3_angle
        if servo3_angle > 0:
            servo3_angle -= 5
            if servo3_angle < 0:
                servo3_angle = 0
            dc3 = servo3_angle_to_duty_cycle(servo3_angle)
            pwm3.ChangeDutyCycle(dc3)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo3_angle, dc3))
            time.sleep(0.2)
            

    def on_left_arrow_press(self):
        global servo4_angle
        if servo4_angle > 0:
            servo4_angle -= 5
            if servo4_angle < 1:
                servo4_angle = 1
            dc4 = servo4_angle_to_duty_cycle(servo4_angle)
            pwm4.ChangeDutyCycle(dc4)
            print('角度={: >3}, 工作週期={:.2f}'.format(servo4_angle, dc4))
            time.sleep(0.2)
            

    def on_left_right_arrow_release(self):
        global servo4_angle
        pwm4.ChangeDutyCycle(servo4_angle_to_duty_cycle(servo4_angle))

    def on_right_arrow_press(self):
        global servo4_angle
        if servo4_angle < 180:
            servo4_angle += 5
            if servo4_angle > 180:
                servo4_angle = 180
            dc4 = servo4_angle_to_duty_cycle(servo4_angle)
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
        while i == 0 :
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