from pyPS4Controller.controller import Controller
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np

class DC_Motor:
    def __init__(self, ena, in1, in2, enb, in3, in4, enc, in5, in6, end, in7, in8):
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

    def right_speed(self, speed):
        self.ENA_SPEED.ChangeDutyCycle(speed)
        self.END_SPEED.ChangeDutyCycle(speed)

    def left_speed(self, speed):
        self.ENB_SPEED.ChangeDutyCycle(speed)
        self.ENC_SPEED.ChangeDutyCycle(speed)

    def speed(self, speed):
        self.right_speed(speed)
        self.left_speed(speed)

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

class Auto:
    def rgiht(i):
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
                        time.sleep(0.18)
                        print('green light')
                        i = 3
                        cv2.destroyAllWindows()
                        break

            if i == 3:
                cap.release()
                print('turn left')
                motor.turn_left()
                time.sleep(0.24)

                print('forward')
                motor.forward()
                time.sleep(0.9)

                print('turn right')
                motor.turn_right()
                time.sleep(0.35)

                print('forward')
                motor.forward()
                time.sleep(0.905)

                print('turn left')
                motor.turn_left()
                time.sleep(0.224)
                motor.forward()
                time.sleep(0.154)
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

    def left(k):
        while k == 1:
            if k == 1:
                motor.forward()
                print('forward')
                time.sleep(0.9)
                motor.stop()
                k = 2

            if k == 2:
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
                        time.sleep(0.18)
                        print('green light')
                        k = 3
                        cv2.destroyAllWindows()
                        break

            if k == 3:
                cap.release()
                print('turn right')
                motor.turn_right()
                time.sleep(0.24)

                print('forward')
                motor.forward()
                time.sleep(0.9)

                print('turn left')
                motor.turn_left()
                time.sleep(0.35)

                print('forward')
                motor.forward()
                time.sleep(0.945)

                print('turn right')
                motor.turn_right()
                time.sleep(0.215)
                motor.forward()
                time.sleep(0.172)
                motor.stop()
                k = 4

            if k == 4:
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
                        print('green light')
                        k = 5
                        cv2.destroyAllWindows()
                        break

            if k == 5:
                cap.release()
                print('forward')
                motor.forward()
                time.sleep(0.65)
                print('final')
                motor.stop()
                k = 6

            elif k == 6:
                print("forward")
                time.sleep(0.42)
                print("stop")
                k = 0
                break
        while k == 0 :
            break

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
motor = DC_Motor(11, 13, 15, 33, 35, 37, 36, 38, 40, 8, 10, 12)
motor_speed = 100
motor.speed(motor_speed)
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
        pass
    def on_L1_release(self):
        pass

    def on_L2_press(self, value):
        print("Auto_left", value)
        if value > 1000 :
            Auto.left(1)

    def on_L2_release(self):
        print("on_L2_release")

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        pass

    def on_R2_press(self, value):
        print("Auto_right ", value)
        if value > 1000 :
            Auto.rgiht(1)

    def on_R2_release(self):
        print("on_R2_release")

    def on_up_arrow_press(self):
        print("on_up_arrow_press")

    def on_up_down_arrow_release(self):
        print("on_up_down_arrow_release")

    def on_down_arrow_press(self):
        print("on_down_arrow_press")

    def on_left_arrow_press(self):
        print("on_left_arrow_press")

    def on_left_right_arrow_release(self):
        print("on_left_right_arrow_release")

    def on_right_arrow_press(self):
        print("on_right_arrow_press")

    def on_L3_up(self, value):
        print("on_L3_up: ", value)

    def on_L3_down(self, value):
        print("on_L3_down: ", value)

    def on_L3_left(self, value):
        print("on_L3_left: ", value)

    def on_L3_right(self, value):
        print("on_L3_right: ", value)

    def on_L3_release(self):
        print("on_L3_release")

    def on_R3_up(self, value):
        print("on_R3_up: ", value)

    def on_R3_down(self, value):
        print("on_R3_down: ", value)

    def on_R3_left(self, value):
        print("on_R3_left: ", value)

    def on_R3_right(self, value):
        print("on_R3_right: ", value)

    def on_R3_release(self):
        print("on_R3_release")

    def on_options_press(self):
        print('on_options_press')

    def on_options_release(self):
        print("on_options_release")


# now make sure the controller is paired over the Bluetooth and turn on the listener
MyController(interface="/dev/input/js0").listen()