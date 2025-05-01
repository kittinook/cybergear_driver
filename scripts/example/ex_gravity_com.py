#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import math

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        timer_period = 0.01  # publish ทุก 1 วินาที
        self.latest_measurements = {}  # จาก topic "joint_states"

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")

    def joint_state_callback(self, msg: JointState):

        for i, name in enumerate(msg.name):
            try:
                motor_id = int(name.split('_')[-1])
            except ValueError:
                continue
            self.latest_measurements[motor_id] = {
                'position': msg.position[i],
                'velocity': msg.velocity[i],
                'current': msg.effort[i]
            }

    def timer_callback(self):
        msg = MotorControlGroup()
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
        torque_com = 0.0
        m = 0.446
        g = 9.81
        l = 0.2
        if self.latest_measurements != {}:
            theta = self.latest_measurements[1]["position"]
            torque_com = m * g * l * math.sin(theta)
            print(torque_com)
            pass
        
        # ตัวอย่างคำสั่งสำหรับ Motor 1 (operation mode)
        motor1 = MotorControl()
        motor1.motor_id = 1
        motor1.control_mode = 0  # Operation mode
        motor1.set_point.velocity = 0.0
        motor1.set_point.position = 0.0
        motor1.set_point.effort = torque_com
        motor1.set_point.kp = 0.0
        motor1.set_point.kd = 0.0


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
