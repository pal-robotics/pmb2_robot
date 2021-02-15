# Copyright (c) 2021 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
import os

from launch import Substitution
from launch.substitutions import LaunchConfiguration
from launch_pal.param_utils import merge_param_files
import launch

from typing import List
from typing import Text




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
        return 'potato name'

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'potat describe'

    def get_motor_config_path(self):
        return os.path.join(get_package_share_directory(
            'pal_deployer_cfg_tiago'), 'config', 'device', 'motor', '')

    def get_sensor_config_path(self):
        return os.path.join(get_package_share_directory(
            'pal_deployer_cfg_tiago'), 'config', 'device', 'sensor', '')

    def get_wheel_motor_files(self, context: launch.LaunchContext):
        # Read the provided variable for model name
        wheel_model = read_launch_argument('wheel_model', context)
        return [self.get_motor_config_path() + 'wheel_left_motor_{}.yaml'.format(wheel_model),
                self.get_motor_config_path() + 'wheel_right_motor_{}.yaml'.format(wheel_model)]

    def get_device_files(self, context: launch.LaunchContext):
        # Read the provided variable for model name
        wheel_model = read_launch_argument('wheel_model', context)
        sensors = []
        motors = ['wheel_left_motor_{}'.format(wheel_model),
                  'wheel_right_motor_{}'.format(wheel_model)]
        arm = read_launch_argument('arm', context)
        if arm == 'True':
            wrist_model = read_launch_argument('wrist_model', context)
            if wrist_model == 'wrist-2010':
                wrist_suffix = '_wrist_2010'
            else:
                wrist_suffix = '_wrist_2017'

            motors += ['arm_1_motor', 'arm_2_motor', 'arm_3_motor', 'arm_4_motor',
                       'arm_5_motor' + wrist_suffix,
                       'arm_6_motor' + wrist_suffix,
                       'arm_7_motor' + wrist_suffix]

            ft_sensor_model = read_launch_argument('ft_sensor_model', context)
            if ft_sensor_model == 'schunk-ft':
                sensors = ['fts' + wrist_suffix]
        full_path_list = []
        for motor in motors:
            full_path_list.append(
                [self.get_motor_config_path() + motor + '.yaml', 'actuators'])

        for sensor in sensors:
            full_path_list.append(
                [self.get_sensor_config_path() + sensor + '.yaml', 'sensors'])

        return full_path_list

    def perform(self, context: launch.LaunchContext) -> Text:

        yaml_files = []
        yaml_files += self.get_device_files(context)
        return merge_param_files(yaml_files)
