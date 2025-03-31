import yaml

def load_config(config_file: str) -> dict:
    """
    โหลด configuration จากไฟล์ YAML
    """
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config
