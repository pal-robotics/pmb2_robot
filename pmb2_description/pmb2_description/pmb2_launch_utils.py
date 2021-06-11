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

from launch.actions import DeclareLaunchArgument


def get_tiago_base_hw_arguments(
        wheel_model=False,
        laser_model=False,
        courier_rgbd_sensors=False,
        default_wheel_model="moog",
        default_laser_model="sick-571",
        default_courier_rgbd_sensors="False"):
    """
    Return TIAGo Base Hardware arguments.

    Returns a list of the requested hardware LaunchArguments for tiago base
    The default value can be configured passing an argument called default
    NAME_OF_ARGUMENT

    example:
        LaunchDescription([*get_tiago_base_hw_arguments(
                                wheel_model=True, laser_model=True,
                                default_laser_model='sick-571')])
    """
    args = []
    if wheel_model:
        args.append(DeclareLaunchArgument(
            'wheel_model',
            default_value=default_wheel_model,
            description='Wheel model, ', choices=["nadia", "moog"]))
    if laser_model:
        args.append(
            DeclareLaunchArgument(
                'laser_model',
                default_value=default_laser_model,
                description='Base laser model. ',
                choices=["no-laser", "sick-571", "sick-561", "sick-551", "hokuyo"]))
    if courier_rgbd_sensors:
        args.append(
            DeclareLaunchArgument(
                'courier_rgbd_sensors',
                default_value=default_courier_rgbd_sensors,
                description='Whether the base has RGBD sensors or not. ',
                choices=["True", "False"]))
    return args
