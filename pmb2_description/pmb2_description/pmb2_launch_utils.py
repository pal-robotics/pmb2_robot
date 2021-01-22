from pmb2_description.launch_utils import DeclareLaunchArgumentChoice

def get_tiago_base_hw_arguments(wheel_model=False,
                                laser_model=False,
                                rgbd_sensors=False,
                                **kwargs):
    """ TIAGo Base Hardware arguments helper function
        Returns a list of the requested hardware LaunchArguments for tiago base
        The default value can be configured passing an argument called default_NAME_OF_ARGUMENT

        example:
            LaunchDescription([*get_tiago_base_hw_arguments(wheel_model=True, laser_model=True, default_laser_model='sick-571')])

    """
    args = []
    if wheel_model:
        args.append(DeclareLaunchArgumentChoice(
            'wheel_model',
            default_value=kwargs.get("default_wheel_model"),
            description='Wheel model, ', arg_choices=["nadia", "moog"]))
    if laser_model:
        args.append(
            DeclareLaunchArgumentChoice(
                'laser_model',
                default_value=kwargs.get("default_laser_model"),
                description='Base laser model. ', arg_choices=["none", "sick-571", "ssick-561", "sick-551", "hokuyo"]))
    if rgbd_sensors:
        args.append(
            DeclareLaunchArgumentChoice(
                'rgbd_sensors',
                default_value=kwargs.get("default_rgbd_sensors"),
                description='Whether the base has RGBD sensors or not. ', arg_choices=["True", "False"]))
    return args


def get_tiago_hw_arguments(arm=False,
                           wrist_model=False,
                           end_effector_model=False,
                           ft_sensor_model=False,
                           **kwargs):
    """ TIAGo Base Hardware arguments helper function
        Returns a list of the requested hardware LaunchArguments for tiago base
        The default value can be configured passing an argument called default_NAME_OF_ARGUMENT

        example:
            LaunchDescription([*get_tiago_hw_arguments(wheel_model=True, laser_model=True, default_laser_model='sick-571')])

    """
    args = get_tiago_base_hw_arguments(
        rgbd_sensors=False,
        **kwargs)  # RGBD on top of base are impossible if torso is installed
    if arm:
        args.append(DeclareLaunchArgumentChoice(
            'arm',
            default_value=kwargs.get("default_arm"),
            description='Whether TIAGo has arm or not. ', arg_choices=["True", "False"]))
    if wrist_model:
        args.append(
            DeclareLaunchArgumentChoice(
                'wrist_model',
                default_value=kwargs.get("default_wrist_model"),
                description='Wrist model. ', arg_choices=["wrist-2010", "wrist-2017"]))
    if end_effector_model:
        args.append(
            DeclareLaunchArgumentChoice(
                'end_effector_model',
                default_value=kwargs.get("default_end_effector_model"),
                description='Wrist model. ', arg_choices=["pal-gripper", "pal-hey5", "schunk-wsg", "custom", "False"]))
    if ft_sensor_model:
        args.append(
            DeclareLaunchArgumentChoice(
                'ft_sensor_model',
                default_value=kwargs.get("default_ft_sensor_model"),
                description='Wrist model. ', arg_choices=["schunk-ft", "False"]))
    return args

