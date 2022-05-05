# Copyright 2022 PAL Robotics S.L.
# All Rights Reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription([
        include_launch_py_description(
            'robot_control', ['launch', 'robot_control.launch.py'],
            launch_arguments={
              'config_pkg': 'pal_deployer_cfg_pmb2',
            }.items(),
        ),
        include_launch_py_description(
            'pmb2_bringup', ['launch', 'pmb2_bringup.launch.py']
        ),
        include_launch_py_description(
            'pmb2_2dnav', ['launch', 'pmb2_nav_bringup.launch.py'],
            launch_arguments={
                'use_sim_time': 'False',
                'use_rviz': 'False',
                }.items()),
    ])

    return ld
