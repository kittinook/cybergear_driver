import time
import can
import math
from cybergear import CyberGear, uint_to_float
from struct import pack, unpack, iter_unpack

def get_feedback(cg, motor_id, echo=False):
    """
    ดึง feedback จากมอเตอร์ผ่านคำสั่ง type2
    แปลงค่าที่อ่านได้ออกเป็น:
      - position: ช่วง -4π ถึง +4π
      - velocity: ช่วง -30 ถึง +30
      - current:  ช่วง -12 ถึง +12
    """
    r = cg.type2(motor_id, echo=echo)
    if r is not None:
        alarm_code = (r[0] >> 16) & 0xff
        fb = r[1]
        position = uint_to_float(fb[0], -4*math.pi, 4*math.pi)
        velocity = uint_to_float(fb[1], -30, 30)
        current  = uint_to_float(fb[2], -12, 12)
        return alarm_code, position, velocity, current
    return None

def command_motor(cg, motor_id, target_value, echo=False):
    """
    สั่งงานมอเตอร์โดยเลือกโหมดที่ต้องการ:
      - "MIT" (0) : 
      - "position (1)" : ส่งค่า target ไปที่ loc_ref
      - "velocity (2)": ส่งค่า target ไปที่ spd_ref
      - "current (3)":  ส่งค่า target ไปที่ iq_ref
    """

    mode = target_value.control_mode
    set_point = target_value.set_point
    if mode == 0:
        # success = cg.set_item_value(motor_id, "loc_ref", target_value, echo=echo)
        success = cg.type1_si( motor_id, set_point.effort, set_point.position, set_point.velocity, set_point.kp, set_point.kd, echo=False)
        if success:
            print(f"Motor {motor_id} commanded in MIT mode with value {target_value}")
        return success
    if mode == 1: # OK
        success = cg.set_item_value(motor_id, "loc_ref", set_point.position, echo=echo)
        if success:
            print(f"Motor {motor_id} commanded in POSITION mode with value {target_value}")
        return success
    elif mode == 2: # OK
        success = cg.set_item_value(motor_id, "spd_ref", set_point.velocity, echo=echo)
        if success:
            print(f"Motor {motor_id} commanded in VELOCITY mode with value {target_value}")
        return success
    elif mode == 3: # OK
        success = cg.set_item_value(motor_id, "iq_ref", set_point.effort, echo=echo)
        if success:
            print(f"Motor {motor_id} commanded in CURRENT mode with value {target_value}")
        return success
    else:
        print("Invalid mode specified. Use 'position', 'velocity', or 'current'.")
        return False

class CyberGearController:
    """
    Controller สำหรับจัดการมอเตอร์หลายตัว โดยอ่าน configuration จากไฟล์ YAML
    """
    def __init__(self, config: dict, echo=False):
        self.config = config
        self.echo = echo
        
        can_config = config.get("can", {})
        self.bus = can.Bus(interface=can_config.get("interface", "canalystii"),
                           channel=can_config.get("channel", 0),
                           bitrate=can_config.get("bitrate", 1000000))
        self.cg = CyberGear(self.bus)
        
        self.motors = config.get("motors", [])
        
        self.control_config = config.get("control", {})

    def setup_motors(self):
        for motor in self.motors:
            motor_id = motor.get("id")
            control_mode = motor.get("control_mode", 2)
            limit_spd = motor.get("limit_spd", 5.0)
            limit_cur = motor.get("limit_cur", 2.0)
            
            self.cg.type4(motor_id, fault=False, echo=self.echo)
            
            self.cg.set_item_value(motor_id, "run_mode", control_mode, echo=self.echo)
            self.cg.set_item_value(motor_id, "limit_spd", limit_spd, echo=self.echo)
            self.cg.set_item_value(motor_id, "limit_cur", limit_cur, echo=self.echo)
            
            self.cg.type3(motor_id, echo=self.echo)
            print(f"Motor {motor_id} setup completed, mode: {control_mode}.")

    def command_all(self, mode: str, target_value: float):
        for motor in self.motors:
            motor_id = motor.get("id")
            command_motor(self.cg, motor_id, mode, target_value, echo=self.echo)

    def get_joint_states(self):
        joint_states = {}
        for motor in self.motors:
            motor_id = motor.get("id")
            fb = get_feedback(self.cg, motor_id, echo=self.echo)
            if fb is not None:
                _, pos, vel, curr = fb
                joint_states[motor_id] = {
                    "position": pos,
                    "velocity": vel,
                    "current": curr
                }
            else:
                joint_states[motor_id] = None
        return joint_states
    
    def create_item_command_frame(self, motor_id, field_name, target_value, echo=False):
        field = next((f for f in self.cg._fields_ if f["name"] == field_name), None)
        if field is None:
            if echo:
                print(f"Field {field_name} not found!")
            return None
        code = field["code"]
        fmt = field["format"]
        
        if fmt != 'f':
            if echo:
                print("Field format not supported in batch command.")
            return None


        try:
            data = pack("<Hxxf", code, target_value)
        except Exception as e:
            if echo:
                print(f"Packing error for motor {motor_id}: {e}")
            return None

        arbitration_id = (motor_id & 0xff) | (0 << 8) | ((18 & 0x1f) << 24)
        frame = can.Message(arbitration_id=arbitration_id,
                            data=data,
                            is_extended_id=True)
        if echo:
            print(f"Created frame for motor {motor_id}: ID=0x{frame.arbitration_id:08x} Data={frame.data.hex()}")
        return frame

    def create_can_frame(self, motor_id, mode, target_value, master_id=0xFD):
        def float_to_uint(x: float, x_min: float, x_max: float):
            if x > x_max:
                x = x_max
            elif x < x_min:
                x = x_min
            return int((x - x_min) * 65535 / (x_max - x_min))
        
        
        mode_dict = {'position': 1, 'velocity': 2, 'torque': 3}

        cmd = mode_dict.get(mode, 0) 

        if mode == 'position':
            target_int = float_to_uint(target_value, -math.pi, math.pi)
        elif mode == 'velocity':
            target_int = float_to_uint(target_value, -20, 20) 
        elif mode == 'torque':
            target_int = float_to_uint(target_value, -5, 5)   
        else:
            target_int = 0

        data = pack('>Hxxxxxx', target_int) 

        arbitration_id = ((cmd & 0x1F) << 24) | ((master_id & 0xFFFF) << 8) | (motor_id & 0xFF)

        frame = can.Message(arbitration_id=arbitration_id,
                            data=data,
                            is_extended_id=True)

        return frame
                            
    def send_group_command1(self, commands: dict, mode: str):
        field_map = {
            "position": "loc_ref",
            "velocity": "spd_ref",
            "current": "iq_ref"
        }
        field_name = field_map.get(mode.lower())
        if field_name is None:
            print("Invalid mode specified. Use 'position', 'velocity', or 'current'.")
            return

        frames = []
        for motor in self.motors:
            motor_id = motor.get("id")
            target_value = commands.get(motor_id)
            if target_value is None:
                continue
            frame = self.create_item_command_frame(motor_id, field_name, target_value, echo=self.echo)
            if frame is not None:
                frames.append(frame)
        self.cg.send_batch(frames)


    def send_group_command(self, commands: dict):
        for motor_id, target_value in commands.items():
            command_motor(self.cg, motor_id, target_value, echo=self.echo)

    def shutdown(self):
        self.cg.shutdown()
