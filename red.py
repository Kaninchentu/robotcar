import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_sum = 0
        self.last_error = 0

    def control(self, error, dt):
        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.Kp * error + self.Ki * self.error_sum + self.Kd * derivative

class MotorDriver:
    I2C_ADDR = 0x34
    MOTOR_TYPE_ADDR = 0x14
    MOTOR_ENCODER_POLARITY_ADDR = 0x15
    MOTOR_FIXED_PWM_ADDR = 0x1F
    MOTOR_FIXED_SPEED_ADDR = 0x33
    MOTOR_ENCODER_TOTAL_ADDR = 0x3C
    
    # MOTOR_TYPE_WITHOUT_ENCODER = 0
    MOTOR_TYPE_TT = 1
    # MOTOR_TYPE_N20 = 2
    # MOTOR_TYPE_JGB37_520_12V_110RPM = 3


    def __init__(self, bus):
        self.dev = I2CDevice(bus, self.I2C_ADDR)
        self.set_speed_all([0, 0, 0, 0])


    def set_motor_type(self, value):
        self.dev.write_byte(self.MOTOR_TYPE_ADDR, value)


    def set_encoder_polarity(self, value):
        self.dev.write_byte(self.MOTOR_ENCODER_POLARITY_ADDR, value)


    def set_speed(self, idx, speed):
        self.dev.write_byte(self.MOTOR_FIXED_SPEED_ADDR + idx - 1, speed)


    def set_speed_all(self, speed_list):
        self.dev.write_bytes(self.MOTOR_FIXED_SPEED_ADDR, speed_list)



def detect_red_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    return mask

def get_line_position(mask):
    moments = cv2.moments(mask)
    if moments['m00'] != 0:
        cX = moments['m10'] / moments['m00']
        position = (cX - mask.shape[1] // 2) / (mask.shape[1] // 2)
        return position
    else:
        return None

motor = MotorDriver()
pid = PIDController(0.5, 0.1, 0.2)
base_speed = 10
max_speed = 10

last_time = time.time()
while cap.isOpened():
    ret, frame = cap.read()
    mask = detect_red_line(frame)
    line_position = get_line_position(mask)

    if line_position is not None:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        speed_correction = pid.control(line_position, dt)

        speed_left = min(max(base_speed - speed_correction, -max_speed), max_speed)
        speed_right = min(max(base_speed + speed_correction, -max_speed), max_speed)
        motor.set_speed(speed_left, speed_right)

    cv2.imshow('Red line', mask)
    key = cv2.waitKey(1)
    if key & 0x00FF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
