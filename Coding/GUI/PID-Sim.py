import time
import random
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, sample_time):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        
        self.prev_error = 0
        self.integral = 0
        
        self.last_time = None

    def update(self, feedback_value):
        # Calculate error
        error = self.setpoint - feedback_value
        
        # Calculate time since last update
        current_time = time.time()
        if self.last_time is None:
            delta_time = self.sample_time
        else:
            delta_time = current_time - self.last_time
        self.last_time = current_time
        
        # Calculate integral
        self.integral += error * delta_time
        
        # Calculate derivative
        derivative = (error - self.prev_error) / delta_time
        
        # Calculate PID output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Update previous error
        self.prev_error = error
        
        return output

# Function to simulate sensor data (random noise for illustration)
def simulate_sensor_data():
    noise = random.uniform(-5, 5)  # Simulate noise in sensor data
    return target_velocity + noise   # Simulate sensor reading around target velocity

# Define constants
target_velocity = 40  # Desired wheel velocity (arbitrary units)
P = 1                  # Proportional term
I = 5                  # Integral term
D = 0.05               # Derivative term

# Initialize PID controller
pid = PIDController(Kp=P, Ki=I, Kd=D, setpoint=target_velocity, sample_time=0.1)

# Lists to store data for plotting
time_data = []
setpoint_data = []
current_velocity_data = []
output_data = []

# Simulation loop
for i in range(50):  # Simulate for 500 iterations
    # Simulate sensor data
    current_velocity = simulate_sensor_data()

    # Update PID controller with current velocity
    pwm_output = pid.update(current_velocity)

    # Store data for plotting
    time_data.append(i)
    setpoint_data.append(target_velocity)
    current_velocity_data.append(current_velocity)
    output_data.append(pwm_output)

    # Delay for visualization (not necessary for actual implementation)
    time.sleep(0.1)

# Plotting
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time_data, setpoint_data, label='Setpoint', linestyle='--')
plt.plot(time_data, current_velocity_data, label='Current Velocity')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity Control')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_data, output_data, label='Output')
plt.xlabel('Time')
plt.ylabel('Output')
plt.title('PID Output')
plt.legend()

plt.tight_layout()
plt.show()
