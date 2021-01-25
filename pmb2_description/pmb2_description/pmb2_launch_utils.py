from launch.actions  import DeclareLaunchArgument

def get_tiago_base_hw_arguments(wheel_model=False,
                                laser_model=False,
                                rgbd_sensors=False,
                                default_wheel_model='moog',
                                default_laser_model='sick-571',
                                default_rgbd_sensors=False #Maybe this needs to be a string
                                ):
    """ TIAGo Base Hardware arguments helper function
        Returns a list of the requested hardware LaunchArguments for tiago base
        The default value can be configured passing an argument called default_NAME_OF_ARGUMENT

        example:
            LaunchDescription([*get_tiago_base_hw_arguments(wheel_model=True, laser_model=True, default_laser_model='sick-571')])

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
                description='Base laser model. ', choices=["sick-571", "sick-561", "sick-551", "hokuyo"]))
    if rgbd_sensors:
        args.append(
            DeclareLaunchArgument(
                'rgbd_sensors',
                default_value=default_rgbd_sensors,
                description='Whether the base has RGBD sensors or not. ', choices=["True", "False"]))
    return args


def get_tiago_hw_arguments(arm=False,
                           wrist_model=False,
                           end_effector_model=False,
                           ft_sensor_model=False,
                           default_arm=True,
                           default_wrist_model="wrist-2017",
                           default_end_effector_model="pal-hey5",
                           default_ft_sensor_model="schunk-ft",
                           **kwargs):
    """ TIAGo Hardware arguments helper function
        Returns a list of the requested hardware LaunchArguments for tiago 
        The default value can be configured passing an argument called default_NAME_OF_ARGUMENT

        Check get_tiago_hw_arguments for more options

        example:
            LaunchDescription([*get_tiago_hw_arguments(wheel_model=True, laser_model=True, default_laser_model='sick-571')])

    """
    args = get_tiago_base_hw_arguments(
        rgbd_sensors=False,
        **kwargs)  # RGBD on top of base are impossible if torso is installed
    if arm:
        args.append(DeclareLaunchArgument(
            'arm',
            default_value=default_arm,
            description='Whether TIAGo has arm or not. ', choices=["True", "False"]))
    if wrist_model:
        args.append(
            DeclareLaunchArgument(
                'wrist_model',
                default_value=default_wrist_model,
                description='Wrist model. ', choices=["wrist-2010", "wrist-2017"]))
    if end_effector_model:
        args.append(
            DeclareLaunchArgument(
                'end_effector_model',
                default_value=default_end_effector_model,
                description='Wrist model. ', choices=["pal-gripper", "pal-hey5", "schunk-wsg", "custom", "False"]))
    if ft_sensor_model:
        args.append(
            DeclareLaunchArgument(
                'ft_sensor_model',
                default_value=default_ft_sensor_model,
                description='Wrist model. ', choices=["schunk-ft", "False"]))
    return args

