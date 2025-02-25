import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.pre_error = 0
        self.integral = 0

    def update(self, ref, current_value, dt):
        error = ref - current_value
        self.integral += error * dt
        derivative = (error - self.pre_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # 更新上一次误差
        self.pre_error = error
        return output

# 模拟小车运动
def simulate_car(pid, target_speed, initial_speed, dt, total_time):
    time_points = []
    speed_points = []
    current_speed = initial_speed
    time = 0

    while time < total_time:
        control_output = pid.update(target_speed, current_speed, dt)
        current_speed += control_output * dt
        time_points.append(time)
        speed_points.append(current_speed)
        time += dt

    return time_points, speed_points

Kp = 1
Ki = 0.8
Kd = 0.1
# 创建PID对象
pid = PIDController(Kp, Ki, Kd)
target_speed = 10
initial_speed = 0.0
dt = 0.01
total_time = 20

time_points, speed_points = simulate_car(pid, target_speed, initial_speed, dt, total_time)

# 绘制
plt.plot(time_points, speed_points, label = 'Actual Speed')
plt.axhline(y=target_speed, color='r', linestyle='--', label='Target Speed') # 水平直线会绘制在 y 轴坐标值等于 target_speed 的位置
plt.xlabel('Time(s)')
plt.ylabel('Speed(m/s)')
plt.title('Car Speed Tracking with PID Control')
plt.legend()
plt.grid(True) # 添加网格线
plt.show()
