from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import copy
import os

from launch import Substitution, SomeSubstitutionsType, LaunchContext
from launch.substitutions import LaunchConfiguration
# import here to avoid loop
from launch.utilities import normalize_to_list_of_substitutions
import launch
import glob
import yaml
import tempfile

from typing import Dict
from typing import List
from typing import Text
from typing import Optional


class DeclareLaunchArgumentChoice(DeclareLaunchArgument):
    """
    Action that declares a launch argument with a limited range of choices
    """
    def __init__(
        self,
        name: Text,
        *,
        default_value: Optional[SomeSubstitutionsType] = None,
        description: Text = 'no description given',
        arg_choices: List[Text],
        **kwargs
    ) -> None:
        """Create a DeclareLaunchArgumentChoice action."""
        expanded_description = description + \
            " Valid choices are: " + str(arg_choices)
        super().__init__(name, default_value=default_value,
                         description=expanded_description, **kwargs)
        self.__arg_choices = arg_choices

    def execute(self, context: LaunchContext):
        super().execute(context)
        value = context.launch_configurations[self.name]
        if value not in self.__arg_choices:
            error_msg = "Argument '{}' provided value '{}' is not valid. Valid options are: {}".format(
                self.name, value, self.__arg_choices)
            # Argument not already set and no default value given, error.
            self._DeclareLaunchArgument__logger.error(error_msg)
            raise RuntimeError(error_msg)



def _merge_dictionaries(dict1, dict2):
    """
    Recursive merge dictionaries.

    :param dict1: Base dictionary to merge.
    :param dict2: Dictionary to merge on top of base dictionary.
    :return: Merged dictionary
    """
    for key, val in dict1.items():
        if isinstance(val, dict):
            dict2_node = dict2.setdefault(key, {})
            _merge_dictionaries(val, dict2_node)
        else:
            if key not in dict2:
                dict2[key] = val

    return dict2


def insert_ros_param_prefix(data, prefix):
    if type(data) != dict:
        return data

    for k in data.keys():
        if k  == "ros__parameters":
            d = {}
            d[prefix] = copy.deepcopy(data[k])
            data[k] = d
        else:
            data[k] = insert_ros_param_prefix(data[k], prefix)
    return data

def merge_param_files(yaml_files):
    """
      Substitution in ROS2 launch can only return a string. The way to combine multiple parameter files is to create a single temporary file and return the path to it, this path is passed as the "parameters" argument of a Node

      yaml_files is a list of either paths to yaml files (strings), or pairs of two strings (path, prefix), 
      so the file is loaded inside the provided prefix, inside the ros__parameters field
    """
    concatenated_dict = {}
    for e in yaml_files:
        if type(e) == str:
            yaml_file = e
            prefix = None
        else:
            yaml_file = e[0]
            prefix = e[1]
        data = yaml.safe_load(open(yaml_file, 'r'))
        if prefix:
            data = insert_ros_param_prefix(data, prefix)

        _merge_dictionaries(concatenated_dict, data)
        # Second arg takes precedence on merge, and result is stored there
        concatenated_dict = data
    rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
    yaml.dump(concatenated_dict, rewritten_yaml)
    rewritten_yaml.close()
    return rewritten_yaml.name

def read_launch_argument(arg_name, context):
    return launch.utilities.perform_substitutions(context,
            [LaunchConfiguration(arg_name)])

class DeviceYamlParams(Substitution):
    """
    Substitution that modifies the given YAML file.
    Used in launch system
    """

    def __init__(self) -> None:
        super().__init__()
        """
    Construct the substitution
    :param: source_file the original YAML file to modify
    """

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return "potato name"

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'potat describe'

    def get_motor_config_path(self):
        return os.path.join(get_package_share_directory(
            'pal_deployer_cfg_tiago'), "config", "device", "motor", "")

    def get_sensor_config_path(self):
        return os.path.join(get_package_share_directory(
            'pal_deployer_cfg_tiago'), "config", "device", "sensor", "")

    def get_wheel_motor_files(self, context: launch.LaunchContext):
        # Read the provided variable for model name
        wheel_model = read_launch_argument("wheel_model", context)
        return [self.get_motor_config_path() + "wheel_left_motor_{}.yaml".format(wheel_model),
                self.get_motor_config_path() + "wheel_right_motor_{}.yaml".format(wheel_model)]

    def get_device_files(self, context: launch.LaunchContext):
        # Read the provided variable for model name
        wheel_model = read_launch_argument("wheel_model", context)
        sensors = []
        motors = ["wheel_left_motor_{}".format(wheel_model),
                "wheel_right_motor_{}".format(wheel_model)]
        arm = read_launch_argument("arm", context)
        if arm == "True":
            wrist_model = read_launch_argument("wrist_model", context)
            if wrist_model == "wrist-2010":
                wrist_suffix = "_wrist_2010"
            else:
                wrist_suffix = "_wrist_2017"
            
            motors += ["arm_1_motor", "arm_2_motor", "arm_3_motor", "arm_4_motor",
                          "arm_5_motor" + wrist_suffix,
                          "arm_6_motor" + wrist_suffix,
                          "arm_7_motor" + wrist_suffix]

            ft_sensor_model = read_launch_argument("ft_sensor_model", context)
            if ft_sensor_model == "schunk-ft":
                sensors = ["fts" + wrist_suffix]
        full_path_list = []
        for motor in motors:
            full_path_list.append([self.get_motor_config_path() + motor + ".yaml", "actuators"])

        for sensor in sensors:
            full_path_list.append([self.get_sensor_config_path() + sensor + ".yaml", "sensors"])

        return full_path_list

    def perform(self, context: launch.LaunchContext) -> Text:

        yaml_files = []
        yaml_files += self.get_device_files(context)
        return merge_param_files(yaml_files)


def generate_launch_description():
    # Arguments that are will be
    configured_params = DeviceYamlParams()

    return LaunchDescription([
        *get_tiago_hw_arguments(wheel_model=True,
                                arm=True, wrist_model=True, end_effector_model=True, ft_sensor_model=True,
                                default_wheel_model="nadia"),
        Node(
            package='my_robot_tutorials',
            executable='minimal_cpp_node',
            parameters=[configured_params],
            output='screen',
            emulate_tty=True
        )
    ])
