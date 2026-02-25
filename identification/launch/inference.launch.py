from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    ld = LaunchDescription()
    #import the parameters from the yaml file 
    pkg = get_package_share_directory("identification")
    config = PathJoinSubstitution([pkg,"config","parameters.yaml"])

    identification = Node(
        package = 'identification',
        executable = 'identif_VLM.py',
        name = 'identification',
        parameters=[config],
        output = 'screen',
    )

    ld.add_action(identification)

    return ld
