import RPi.GPIO as GPIO


class DC_motor:
    def __init__(self, ena, in1, in2, enb, in3, in4, enc, in5, in6, end, in7, in8):  # 四輪驅動
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

    # Speed
    def right_speed(self, speed):
        self.ENA_SPEED.ChangeDutyCycle(speed)
        self.END_SPEED.ChangeDutyCycle(speed)

    def left_speed(self, speed):
        self.ENB_SPEED.ChangeDutyCycle(speed)
        self.ENC_SPEED.ChangeDutyCycle(speed)

    def speed(self, speed):
        self.right_speed(speed)
        self.left_speed(speed)

    # Move
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
        GPIO.output(self.IN5, False)  # 先測試看一輪動不動得了
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
        GPIO.output(self.IN7, False)  # 先測試一輪動不動得了
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
