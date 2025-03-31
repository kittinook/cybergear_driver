#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from std_msgs.msg import Header

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")

    def timer_callback(self):
        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" 


        motor1 = MotorControl()
        motor1.motor_id = 1
        motor1.control_mode = 0  # Operation mode
        motor1.set_point.velocity = 0.0
        motor1.set_point.position = 0.0
        motor1.set_point.effort = 0.0
        motor1.set_point.kp = 0.01
        motor1.set_point.kd = 0.01

        msg.motor_controls = [motor1]

        self.get_logger().info("Publishing MotorGroupControl message")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlGroupPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
