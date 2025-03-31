#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from std_msgs.msg import Header

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        timer_period = 1.0  # publish ทุก 1 วินาที
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")

    def timer_callback(self):
        # สร้าง message MotorGroupControl
        msg = MotorControlGroup()
        # กำหนด header (ใช้เวลาปัจจุบัน)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # ค่า frame_id แบบ dummy

        # ตัวอย่างสร้างคำสั่งควบคุมสำหรับแต่ละมอเตอร์
        # MotorControl message:
        #   - motor_id: หมายเลขมอเตอร์
        #   - control_mode: 0 = operation, 1 = current, 2 = speed, 3 = position
        #   - parameters: รายการพารามิเตอร์ (Operation: [torque, angle, speed, kp, kd],
        #                                Current: [current_ref],
        #                                Speed: [speed_ref],
        #                                Position: [position_ref])

        # ตัวอย่างคำสั่งสำหรับ Motor 1 (operation mode)
        motor1 = MotorControl()
        motor1.motor_id = 1
        motor1.control_mode = 0  # Operation mode
        motor1.set_point.velocity = 0.0
        motor1.set_point.position = 0.0
        motor1.set_point.effort = 0.0
        motor1.set_point.kp = 0.01
        motor1.set_point.kd = 0.01

        # ตัวอย่างคำสั่งสำหรับ Motor 2 (operation mode)
        # motor2 = MotorControl()
        # motor2.motor_id = 2
        # motor2.control_mode = 2  # Operation mode
        # motor2.set_point.velocity = 0.5
        # motor2.set_point.position = 0.0
        # motor2.set_point.effort = 0.0
        # motor2.set_point.kp = 0.0
        # motor2.set_point.kd = 0.0

        # motor3 = MotorControl()
        # motor3.motor_id = 3
        # motor3.control_mode = 2  # Operation mode
        # motor3.set_point.velocity = 0.5
        # motor3.set_point.position = 0.0
        # motor3.set_point.effort = 0.0
        # motor3.set_point.kp = 0.0
        # motor3.set_point.kd = 0.0

        # ตัวอย่างคำสั่งสำหรับ Motor 3 (current mode)
        # motor3 = MotorControl()
        # motor3.motor_id = 3
        # motor3.control_mode = 1  # Current mode
        # motor3.parameters = [1.5]  # [current_ref]

        # รวมคำสั่งทั้งหมดลงใน motor_controls array
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
