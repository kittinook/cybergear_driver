# import yaml

# def load_config(config_file: str) -> dict:
#     """
#     โหลด configuration จากไฟล์ YAML
#     """
#     with open(config_file, 'r') as f:
#         config = yaml.safe_load(f)
#     return config

import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_config(config_file):
    # หาก config_file ไม่ใช่ absolute path ให้ดึงจาก package share directory
    if not os.path.isabs(config_file):
        pkg_share = get_package_share_directory('cybergear_drive')
        # สมมุติว่าไฟล์ config.yaml อยู่ในโฟลเดอร์ 'config' ภายใน package share
        config_file = os.path.join(pkg_share, 'config', config_file)
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config
