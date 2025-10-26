#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class OmniPWMToVelocity(Node):
    def __init__(self):
        super().__init__('omni_pwm_to_velocity')

        # Parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_length', 0.3)
        self.declare_parameter('wheel_base_width', 0.3)
        self.declare_parameter('pwm_scale_linear', 0.001)   # m/s per PWM unit
        self.declare_parameter('pwm_scale_angular', 0.001)  # rad/s per PWM unit

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base_length').value
        self.W = self.get_parameter('wheel_base_width').value
        self.scale_lin = self.get_parameter('pwm_scale_linear').value
        self.scale_ang = self.get_parameter('pwm_scale_angular').value

        # Subscriber: [surge_pwm, lateral_pwm, yaw_pwm]
        self.sub = self.create_subscription(
            Float64MultiArray,
            'pwm_input',  # expects 3 values: [surge, lateral, yaw]
            self.pwm_callback,
            10
        )

        # Publisher: wheel velocities [FL, FR, BL, BR]
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/omni_wheel_controller/commands',
            10
        )

        self.get_logger().info("Omni PWM → Wheel Velocity node started")

    def pwm_callback(self, msg: Float64MultiArray):
            if len(msg.data) < 3:
                self.get_logger().warn("PWM input must have 3 values: [surge, lateral, yaw]")
                return

            surge_pwm, lateral_pwm, yaw_pwm = msg.data

            # Convert PWM to linear/angular velocity
            vx = surge_pwm * self.scale_lin   # forward/back
            vy = lateral_pwm * self.scale_lin # left/right
            wz = yaw_pwm * self.scale_ang     # yaw

            # Cross (“+”) omni wheel kinematics
            L, r = self.L, self.r
            M = (1 / r) * np.array([
                [1,  0,  L],  # right wheel
                [1,  0, -L],  # left wheel
                [0,  1,  L],  # front wheel
                [0,  1, -L]   # back wheel
            ])

            # Compute wheel velocities
            wheel_vels = np.dot(M, np.array([vx, vy, wz]))

            # Publish
            out = Float64MultiArray()
            out.data = wheel_vels.tolist()
            self.pub.publish(out)

            self.get_logger().debug(
                f"PWM [{surge_pwm:.1f}, {lateral_pwm:.1f}, {yaw_pwm:.1f}] → Wheels {wheel_vels}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = OmniPWMToVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
