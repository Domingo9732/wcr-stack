import os
import yaml
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


class LauncherConfigurator:
    
    def __init__(self):
        self.launcher_share = get_package_share_path('wcr_launcher')
        self._load_configs()
    
    def _load_configs(self):
        launch_params_yaml = os.path.join(self.launcher_share, 'config', 'launch_params.yaml')
        with open(launch_params_yaml, 'r') as f:
            self.launch_params = yaml.safe_load(f) or {}

        robot_params_yaml = os.path.join(self.launcher_share, 'config', 'robot_params.yaml')
        with open(robot_params_yaml, 'r') as f:
            self.robot_params = yaml.safe_load(f) or {}
    
    def get_launch_param(self, key, default=None):
        return self.launch_params.get(key, default)
    
    def get_robot_param(self, key, default=None):
        return self.robot_params.get(key, default)
    
    def get_all_robot_params(self):
        return self.robot_params
    
    def build_robot_description(self):
        """Build robot_description from xacro with all parameters"""
        pkg_share = get_package_share_path('wcr_description')
        urdf_path = os.path.join(pkg_share, 'urdf', self.urdf_file)
        
        xacro_args = [
            "xacro", " ", urdf_path,
            " ", f"variant:={self.variant}",
            " ", f"sim:={self.sim}",
            " ", f"propeller_control:={self.propeller_control}",
            " ", f"namespace:={self.namespace}",
        ]
        
        for key, value in self.get_all_robot_params().items():
            if isinstance(value, bool):
                value = str(value).lower()
            xacro_args.extend([" ", f"{key}:={value}"])
      
        return ParameterValue(
            Command(xacro_args),
            value_type=str
        )
    
    @property
    def namespace(self):
        return self.get_launch_param('namespace', 'wcr')
    
    @property
    def variant(self):
        return self.get_launch_param('variant', 'mock')
    
    @property
    def sim(self):
        return self.get_launch_param('sim', True)
    
    @property
    def propeller_control(self):
        return self.get_launch_param('propeller_control', False)
    
    @property
    def urdf_file(self):
        return self.get_launch_param('urdf_file', 'wcr.urdf.xacro')
    
    @property
    def use_sim_time(self):
        return self.get_launch_param('use_sim_time', True)
    
    @property
    def use_controllers(self):
        return self.get_launch_param('controller', True)
    
    @property
    def world_file(self):
        return self.get_launch_param('world_file', 'robot_world.sdf')
    
    @property
    def spawn_z_height(self):
        return self.get_launch_param('spawn_z_height', 0.2)
    
    @property
    def gz_args(self):
        return self.get_launch_param('gz_args', '-r -v4')
    
    @property
    def controllers_yaml(self):
        return os.path.join(self.launcher_share, 'config', 'wcr_controller.yaml')
