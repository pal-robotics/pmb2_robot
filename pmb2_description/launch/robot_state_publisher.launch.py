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

from launch import LaunchDescription, Substitution, LaunchContext
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.substitutions import Command, PathJoinSubstitution
from typing import List
from typing import Text

from pmb2_description.pmb2_launch_utils import get_tiago_base_hw_arguments


class Pmb2XacroConfigSubstitution(Substitution):
    """Extract the pmb2 hardware args and passes them as xacro variables."""

    def __init__(self) -> None:
        super().__init__()

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return "PMB2 Xacro Config"

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'Parses pmb2 hardware launch arguments into xacro arguments potat describe'

    def perform(self, context: LaunchContext) -> Text:
        """Generate the robot description and return it as a string."""
        laser_model = read_launch_argument("laser_model", context)
        courier_rgbd_sensors = read_launch_argument(
            "courier_rgbd_sensors", context)
        return " laser_model:=" + laser_model + " courier_rgbd_sensors:=" + courier_rgbd_sensors


def generate_launch_description():

    parameters = {'robot_description': Command(
        [
            ExecutableInPackage(package='xacro', executable="xacro"),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('pmb2_description'), 'robots', 'pmb2.urdf.xacro']),
            Pmb2XacroConfigSubstitution()
        ])
    }

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[parameters])
    return LaunchDescription([
        *get_tiago_base_hw_arguments(laser_model=True,
                                     courier_rgbd_sensors=True,
                                     default_laser_model="sick-571",
                                     default_courier_rgbd_sensors="False"),
        rsp
    ])
