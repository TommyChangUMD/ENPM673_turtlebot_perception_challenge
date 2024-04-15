from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions                import Node

from ament_index_python.packages       import get_package_share_directory

import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        get_package_share_directory('enpm673_final_proj'),
        'worlds',
        'enpm673.world'
#        'animated_box.world'
    )

    print (f"world = {world}")

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': world}.items()
    )

    # Add the commands to the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_cmd)
    return ld
    
