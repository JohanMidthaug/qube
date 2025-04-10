import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# Create a PidController class to control the QUBE-servo
class PidController:
    # Initialize necessary parameters for PID-controller
    def __init__(self, kp, ki, kd, reference, velocity_signal, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reference = reference
        self.velocity_signal = velocity_signal
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    # Define the update function, which is called whenever we want to calculate a new signal from the PID-controller
    def update(self, current_position):
        error = self.reference - current_position

        P = self.kp * error

        self.integral += error * self.dt
        self.integral_windup = max(min(self.integral, 5.0), -5.0)
        I = self.ki * self.integral_windup

        derivative = (current_position - self.previous_error) / self.dt if self.dt > 0 else 0
        D = self.kd * derivative

        self.velocity_signal = P + I + D

        self.previous_error = error

        return self.velocity_signal

# Define the PidControllerNode, which will be used to start up the ROS2-node which controls the QUBE-servo
class PidControllerNode(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('qube_controller_node')
        self.get_logger().info("Qube PID Controller Node Initialized")  # Debug line

        # Declare parameters at startup
        self.declare_parameter('kp', 11.0) # 15.0 / 10.0
        self.declare_parameter('ki', 0.0) # 71.43 / 40.0
        self.declare_parameter('kd', 0.0) # 0.7875 / 3.0
        self.declare_parameter('reference', 0.0)

        # Assign the parameters their values, either from callback or from startup
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.reference = self.get_parameter('reference').value

        # Initialize the PID controller
        self.current_velocity = 0.0
        self.current_position = 0.0
        self.pid_controller = PidController(kp=self.kp, ki=self.ki, kd=self.kd, reference=self.reference, velocity_signal=0.0, dt=0.1)

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create a subscriber to get the angle from the motor
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Create a publisher to output the velocity
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Timer to repeatedly compute the PID output
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def joint_state_callback(self, msg):
        # Gets the joint position and velocity from /joint_states topic
        if len(msg.position) > 0 and len(msg.velocity) > 0:
            self.current_position = msg.position[0]
            self.current_velocity = msg.velocity[0]

        #self.get_logger().info(f'Current position: {self.current_position}, current reference: {self.reference}')

    def publish_velocity(self):
        # Use the measured position and compute the output velocity, using pid controller update function
        velocity = self.pid_controller.update(self.current_position)

        # Construct the velocity message, and send it using the publisher
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [velocity]
        self.velocity_publisher.publish(velocity_msg)

        #self.get_logger().info(f'Publishing velocity: {velocity_msg.data}')

    # Function to be able to update PID-parameters while at startup, or while the program is already running
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                if param.value >= 0.0:
                    self.kp = param.value
                    self.get_logger().info(f' kp was set: {self.kp}')
                else:
                    self.get_logger().info(f' kp was not set to: {self.kp}, out of scope')
                    return SetParametersResult(success=False)
            elif param.name == 'ki':
                if param.value >= 0.0:
                    self.ki = param.value
                    self.get_logger().info(f' ki was set: {self.ki}')
                else:
                    self.get_logger().info(f' ki was not set to: {self.ki}, out of scope')
                    return SetParametersResult(success=False)
            elif param.name == 'kd':
                if param.value >= 0.0:
                    self.kd = param.value
                    self.get_logger().info(f' kd was set: {self.kd}')
                else:
                    self.get_logger().info(f' kd was not set to: {self.kd}, out of scope')
                    return SetParametersResult(success=False)
            elif param.name == 'reference':
                if 3.1415 >= param.value >= -3.1415:
                    self.reference = param.value
                    self.get_logger().info(f' reference was set: {self.reference}')
                else:
                    self.get_logger().info(f' reference was not set to: {self.reference}, out of scope')
                    return SetParametersResult(success=False)

        self.pid_controller.kp = self.kp
        self.pid_controller.ki = self.ki
        self.pid_controller.kd = self.kd
        self.pid_controller.reference = self.reference

        self.get_logger().info(f'PID Controller updated with kp={self.kp}, ki={self.ki}, kd={self.kd}, reference={self.reference}')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    # Start an instance of the PidControllerNode()
    pid_controller_node = PidControllerNode()
    # Add a printout for debugging purposes
    pid_controller_node.get_logger().info('Starting PID controller node')
    # Spin the pid_controller node
    rclpy.spin(pid_controller_node)
    # Finally shut it down
    pid_controller_node.destroy_node()
    rclpy.shutdown()

