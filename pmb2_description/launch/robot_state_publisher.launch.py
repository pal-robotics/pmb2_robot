# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

from dataclasses import dataclass
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_pal.robot_arguments import CommonArgs
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from pmb2_description.launch_arguments import PMB2Args
from launch_param_builder import load_xacro
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    wheel_model: DeclareLaunchArgument = PMB2Args.wheel_model
    laser_model: DeclareLaunchArgument = PMB2Args.laser_model
    add_on_module: DeclareLaunchArgument = PMB2Args.add_on_module
    camera_model: DeclareLaunchArgument = PMB2Args.camera_model
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim
    namespace: DeclareLaunchArgument = CommonArgs.namespace


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    launch_description.add_action(
        OpaqueFunction(function=create_robot_description_param)
    )

    # Using ParameterValue is needed so ROS knows the parameter type
    # Otherwise https://github.com/ros2/launch_ros/issues/136
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='both',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'), value_type=str
                )
            }
        ],
        # Provide support to <namespace>/tf and <namespace>/tf_static
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    launch_description.add_action(rsp)

    return


def create_robot_description_param(context, *args, **kwargs):

    xacro_file_path = Path(
        os.path.join(
            get_package_share_directory('pmb2_description'),
            'robots',
            'pmb2.urdf.xacro',
        )
    )

    xacro_input_args = {
        'laser_model': read_launch_argument('laser_model', context),
        'add_on_module': read_launch_argument('add_on_module', context),
        'camera_model': read_launch_argument('camera_model', context),
        'use_sim_time': read_launch_argument('use_sim_time', context),
        'is_public_sim': read_launch_argument('is_public_sim', context),
        'namespace': read_launch_argument('namespace', context),

    }
    robot_description = load_xacro(xacro_file_path, xacro_input_args)

    return [SetLaunchConfiguration('robot_description', robot_description)]
