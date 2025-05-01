#!/usr/bin/env python3

# 1 motor -> 620 hz -> 392 hz
# 2 motor -> 415 hz -> 232 hz
# 3 motor -> 314 hz -> 167 hz
# 4 motor -> 245 hz -> 147 hz
# 5 motor -> 203 hz -> 132 hz
# 6 motor -> 185 hz -> 117 hz

# Log
# 1/04/2025 Add calibration function
# 1/05/2025 Add direction parameter


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger 

from config import load_config
from controller import CyberGearController
from cybergear_interfaces.msg import MotorControlGroup

class CyberGearROS2Node(Node):
    def __init__(self):
        super().__init__('cybergear_node')
        
        config_file = self.declare_parameter('config_file', 'config.yaml').value
        config = load_config(config_file)
        self.get_logger().info(f"Load Current Config:\n{config}")
        
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
        
        self.calibration_srv = self.create_service(Trigger, 'calibrate', self.handle_calibration)
        
        dt = config.get("params")['timer_period_hz']
        
        self.timer = self.create_timer(1/dt, self.timer_callback)  # 10 Hz
        
        
        self.group_command = None
        self.dir = []
        self.joint_name = []
        
        self.motors = config.get("motors", [])
        for i, motor in enumerate(self.motors):
            if motor.get("direction") == -1:
                self.dir.append(-1)
            else:
                self.dir.append(1)
            if motor.get("joint_name") != None:
                self.joint_name.append(motor.get("joint_name"))
            else:
                self.joint_name.append("motor_" + str(i))
        # ตัวแปรสำหรับการ calibrate
        self.is_calibrated = False
        self.offsets = {}   # { motor_id: offset_value }
        self.gear_ratio = 7  # gearbox ratio

    def command_callback(self, msg: MotorControlGroup):
        if len(msg.motor_controls) != len(self.motors):
            self.get_logger().warn("Received group command length does not match number of motors")
            return
        
        command_dict = {}
        for i, motor in enumerate(self.motors):
            if self.is_calibrated:
                # msg.motor_controls[i].set_point.position = (float(self.dir[i])*msg.motor_controls[i].set_point.position) + float(motor.get("offset_pos_1")) + float(motor.get("offset_pos_2"))
                msg.motor_controls[i].set_point.position = (float(self.dir[i])*msg.motor_controls[i].set_point.position) + float(motor.get("offset_pos_2")) + float(motor.get("offset_pos_1"))
            else:
                pass
            motor_id = motor.get("id")
            command_dict[motor_id] = msg.motor_controls[i]
            
        self.group_command = command_dict
        self.get_logger().info(f"Received group command: {self.group_command}")

    def handle_calibration(self, request, response):
        joint_states = self.controller.get_joint_states()
        for motor in self.motors:
            motor_id = motor.get("id")
            state = joint_states.get(motor_id)
            if state and "position" in state:
                self.offsets[motor_id] = state["position"]
            else:
                self.offsets[motor_id] = 0.0
        self.is_calibrated = True
        msg = f"Calibration complete. Offsets: {self.offsets}"
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def timer_callback(self):
        if self.group_command is not None:
            self.controller.send_group_command(self.group_command)
        
        joint_states = self.controller.get_joint_states()
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        
        names = []
        positions = []
        velocities = []
        efforts = [] 
        i = 0
        for motor in self.motors:
            motor_id = motor.get("id")
            # names.append(f"motor_{motor_id}")
            names.append(self.joint_name[i])
            state = joint_states.get(motor_id)
            if state:
                raw_position = state.get("position", 0.0)
                if self.is_calibrated:
                    calibrated_position = (self.dir[i] * ((raw_position - motor.get("offset_pos_1")))) - motor.get("offset_pos_2")
                else:
                    calibrated_position = raw_position

                positions.append(calibrated_position)
                velocities.append(state.get("velocity", 0.0))
                efforts.append(state.get("current", 0.0))
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
            
            i += 1
        
        js_msg.name = names
        js_msg.position = positions
        js_msg.velocity = velocities
        js_msg.effort = efforts
        
        self.joint_state_pub.publish(js_msg)
        # self.get_logger().info("Published joint states")
    
    def destroy_node(self):
        # Shutdown controller ก่อนปิด node
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