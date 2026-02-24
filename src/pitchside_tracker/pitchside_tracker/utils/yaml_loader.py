import yaml
from ament_index_python.packages import get_package_share_directory
import os
def load_yaml():
    # --- Load YAML config ---
    config_path = os.path.join(
        get_package_share_directory('pitchside_tracker'),
        'config',
        'square_pass.yaml'
    )   
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)