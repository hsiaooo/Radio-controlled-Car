import time
import RPi.GPIO as GPIO
from getkey import getkey, keys
import cv2
import numpy as np

class Motor:
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

        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.LOW)

        self.ENA_SPEED = GPIO.PWM(self.ENA, 600)
        self.ENA_SPEED.start(0)
        self.ENA_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(self.ENB, GPIO.OUT, initial=GPIO.LOW)
        self.ENB_SPEED = GPIO.PWM(self.ENB, 600)
        self.ENB_SPEED.start(0)
        self.ENB_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN4, GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(self.ENC, GPIO.OUT, initial=GPIO.LOW)
        self.ENC_SPEED = GPIO.PWM(self.ENC, 600)
        self.ENC_SPEED.start(0)
        self.ENC_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN5, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN6, GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(self.END, GPIO.OUT, initial=GPIO.LOW)
        self.END_SPEED = GPIO.PWM(self.END, 600)
        self.END_SPEED.start(0)
        self.END_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN7, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN8, GPIO.OUT, initial=GPIO.LOW)

    def right_speed(self, speed):
        self.ENA_SPEED.ChangeDutyCycle(speed)
        self.ENC_SPEED.ChangeDutyCycle(speed)

    def left_speed(self, speed):
        self.ENB_SPEED.ChangeDutyCycle(speed)
        self.END_SPEED.ChangeDutyCycle(speed)

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
        GPIO.output(self.IN5, True)
        GPIO.output(self.IN6, False)
        GPIO.output(self.IN7, False)
        GPIO.output(self.IN8, True)

    def turn_right(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, False)
        GPIO.output(self.IN2, True)
        GPIO.output(self.IN3, True)
        GPIO.output(self.IN4, False)
        GPIO.output(self.IN5, False)
        GPIO.output(self.IN6, True)
        GPIO.output(self.IN7, True)
        GPIO.output(self.IN8, False)

    def dir_left(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.ENB, True)
        GPIO.output(self.ENC, True)
        GPIO.output(self.END, True)
        GPIO.output(self.IN1, True)
        GPIO.output(self.IN2, False)
        GPIO.output(self.IN3, False)
        GPIO.output(self.IN4, True)
        GPIO.output(self.IN5, False)
        GPIO.output(self.IN6, True)
        GPIO.output(self.IN7, True)
        GPIO.output(self.IN8, False)

    def dir_right(self):
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
        GPIO.output(self.IN7, False)
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

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
motor = Motor(36, 38, 40, 11, 13, 15, 19, 21, 23, 29, 31, 33)
motor_speed = 80
i=0


while True:

    if i == 1:
        motor.forward()
        print('forward')
        time.sleep(1)
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
                print('green light')
                i = 3
                cv2.destroyAllWindows()
                break

    if i == 3:
        cap.release()
        print('turn left')
        motor.turn_left()
        time.sleep(0.5)

        print('forward')
        motor.forward()
        time.sleep(1)

        print('turn right')
        motor.turn_right()
        time.sleep(0.5)

        print('forward')
        motor.forward()
        time.sleep(1)

        print('turn left')
        motor.turn_left()
        time.sleep(0.5)

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
        time.sleep(0.5)
        print('final')
        i = 0

            
        
    key = getkey()         
    if key == "w":
        print('forward')
        motor.forward()
    if key == "s":
        print('backward')
        motor.backward()
    if key == "e":
        print('turn right')
        motor.turn_right()
        time.sleep(0.1)
        motor.stop()
    if key == "q":
        print('turn left')
        motor.turn_left()
        time.sleep(0.1)
        motor.stop()
    if key == "a":
        print('dir left')
        motor.dir_left()
    if key == "d":
        print('dir right')
        motor.dir_right()
    if key == 'l':
        motor_speed -= 10
        print(motor_speed)
        if motor_speed <= 30:
            motor_speed = 30
    if key == 'h':
        motor_speed += 10
        print(motor_speed)
        if motor_speed >= 100:
            motor_speed = 100
    if key == 'n':
        motor.stop()
    if key == keys.ESCAPE:
        motor.stop()
        break

    if key == 'o':
        print('Auto begin')
        i=1
        
        


