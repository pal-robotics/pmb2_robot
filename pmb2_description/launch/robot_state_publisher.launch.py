from launch import LaunchDescription, Substitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.substitutions import Command, PathJoinSubstitution
from typing import List
from launch import Substitution, SomeSubstitutionsType, LaunchContext
from typing import Text

from pmb2_description.launch_utils import read_launch_argument 
from pmb2_description.pmb2_launch_utils import get_tiago_base_hw_arguments 


class Pmb2XacroConfigSubstitution(Substitution):
    """
    Substitution extracts the pmb2 hardware args and passes them as xacro variables
    Used in launch system
    """

    def __init__(self) -> None:
        super().__init__()
        """
    Construct the substitution
    :param: source_file the original YAML file t
    """

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return "PMB2 Xacro Config"

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'Parses pmb2 hardware launch arguments into xacro arguments potat describe'

    def perform(self, context: LaunchContext) -> Text:
        """
           Generate the robot description and return it as a string
        """

        laser_model = read_launch_argument("laser_model", context)
        rgbd_sensors = read_launch_argument("rgbd_sensors", context)
        return " laser_model:=" + laser_model + " rgbd_sensors:=" + rgbd_sensors

def generate_launch_description():

    parameters={'robot_description': Command(
      [
        ExecutableInPackage(package='xacro', executable="xacro"),
        ' ',
        PathJoinSubstitution([FindPackageShare('pmb2_description'), 'robots', 'pmb2.urdf.xacro']),
        Pmb2XacroConfigSubstitution()
      ])
    }

    rsp = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[parameters])
    return LaunchDescription([
        *get_tiago_base_hw_arguments(laser_model=True,
                                     rgbd_sensors=True,
                                     default_laser_model="sick-571",
                                     default_rgbd_sensors="False"),
        rsp
    ])
