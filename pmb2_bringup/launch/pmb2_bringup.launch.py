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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_pal.robot_arguments import CommonArgs
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.include_utils import include_scoped_launch_py_description
from pmb2_description.launch_arguments import PMB2Args


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

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    default_controllers = include_scoped_launch_py_description(
        pkg_name='pmb2_controller_configuration',
        paths=['launch', 'default_controllers.launch.py'],
        launch_arguments={
            'is_public_sim': launch_args.is_public_sim,
            'namespace': launch_args.namespace,
        }
    )

    launch_description.add_action(default_controllers)

    twist_mux = include_scoped_launch_py_description(
        pkg_name='pmb2_bringup',
        paths=['launch', 'twist_mux.launch.py'],
        launch_arguments={
            'use_sim_time': launch_args.use_sim_time,
            'namespace': launch_args.namespace,
        },
    )

    launch_description.add_action(twist_mux)

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='pmb2_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={
            'wheel_model': launch_args.wheel_model,
            'laser_model': launch_args.laser_model,
            'add_on_module': launch_args.add_on_module,
            'camera_model': launch_args.camera_model,
            'use_sim_time': launch_args.use_sim_time,
            'is_public_sim': launch_args.is_public_sim,
            'namespace': launch_args.namespace,
        },
    )

    launch_description.add_action(robot_state_publisher)

    return
