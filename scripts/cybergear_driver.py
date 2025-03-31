#!/usr/bin/env python3

# 1 motor -> 620 hz -> 392 hz
# 2 motor -> 415 hz -> 232 hz
# 3 motor -> 314 hz -> 167 hz
# 4 motor -> 245 hz -> 147 hz
# 5 motor -> 203 hz -> 132 hz
# 6 motor -> 185 hz -> 117 hz

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from config import load_config
from controller import CyberGearController


from cybergear_interfaces.msg import MotorControlGroup

class CyberGearROS2Node(Node):
    def __init__(self):
        super().__init__('cybergear_node')
        
        config_file = self.declare_parameter('config_file', 'config.yaml').value
        config = load_config(config_file)
        print("Load Current Config")
        print(config)
        
        self.controller = CyberGearController(config, echo=False)
        self.get_logger().info("CyberGearController initialized")
        
        self.controller.setup_motors()
        
        self.mode = self.declare_parameter('control_mode', 'velocity').value
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.command_sub = self.create_subscription(
            MotorControlGroup,
            'motor_group_command',
            self.command_callback,
            10
        )
        
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10 Hz
        
        self.group_command = None
        
        self.motors = config.get("motors", [])

        self.is_calibrate = False
    
    def command_callback(self, msg: MotorControlGroup):
        """
        callback รับ group command จาก topic 'group_command'
        สมมุติว่า message นี้มี array ที่ความยาวเท่ากับจำนวนมอเตอร์
        โดย index ของ array ตรงกับ order ของมอเตอร์ใน config
        """
        if len(msg.motor_controls) != len(self.motors):
            self.get_logger().warn("Received group command length does not match number of motors")
            return
        
        command_dict = {}
        for i, motor in enumerate(self.motors):
            motor_id = motor.get("id")
            command_dict[motor_id] = msg.motor_controls[i]
            
        self.group_command = command_dict
        self.get_logger().info(f"Received group command: {self.group_command}")
    
    def timer_callback(self):
        """
        timer callback:
          - ถ้ามี group command ส่งคำสั่งให้มอเตอร์ผ่าน controller
          - ดึงข้อมูล joint state จากมอเตอร์ทุกตัวและ publish ออกไป
        """
        
        if self.group_command is not None:
            self.controller.send_group_command(self.group_command)
        
        joint_states = self.controller.get_joint_states()
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        
        names = []
        positions = []
        velocities = []
        efforts = []  
        
        for motor in self.motors:
            motor_id = motor.get("id")
            names.append(f"motor_{motor_id}")
            state = joint_states.get(motor_id)
            if state:
                positions.append(state.get("position", 0.0))
                velocities.append(state.get("velocity", 0.0))
                efforts.append(state.get("current", 0.0))
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
        
        js_msg.name = names
        js_msg.position = positions
        js_msg.velocity = velocities
        js_msg.effort = efforts
        
        self.joint_state_pub.publish(js_msg)
        self.get_logger().info("Published joint states")
    
    def destroy_node(self):
        self.controller.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CyberGearROS2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
