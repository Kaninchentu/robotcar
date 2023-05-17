import time

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


# 以下是假设的电机控制代码
class Motor:
    def set_speed(self, speed):
        pass  # 在这里添加你的电机控制代码


def get_line_position():
    pass  # 在这里添加你的图像处理代码，返回线的位置（从-1到1）


# 初始化
motor_left = Motor()
motor_right = Motor()
pid = PIDController(0.5, 0.1, 0.2)  # 你需要调整这些参数以适应你的设备
base_speed = 0.5  # 基本速度
max_speed = 1.0  # 最大速度

# 主循环
last_time = time.time()
while True:
    line_position = get_line_position()
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # PID 控制
    speed_correction = pid.control(line_position, dt)

    # 控制电机速度
    speed_left = min(max(base_speed - speed_correction, -max_speed), max_speed)
    speed_right = min(max(base_speed + speed_correction, -max_speed), max_speed)
    motor_left.set_speed(speed_left)
    motor_right.set_speed(speed_right)
