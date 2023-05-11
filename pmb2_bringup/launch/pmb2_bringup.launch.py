# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():

    # TODO missing equivalent to ROS1 robot_pose_node
    # TODO missing equivalent to tf_lookup, but it may have been legacy from tf1

    default_controllers_launch = include_launch_py_description(
        'pmb2_controller_configuration', ['launch', 'default_controllers.launch.py'])

    twist_mux_launch = include_launch_py_description(
        'pmb2_bringup', ['launch', 'twist_mux.launch.py'])

    pmb2_state_publisher = include_launch_py_description(
        'pmb2_description', ['launch', 'robot_state_publisher.launch.py'])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(default_controllers_launch)
    ld.add_action(twist_mux_launch)
    ld.add_action(pmb2_state_publisher)

    return ld
