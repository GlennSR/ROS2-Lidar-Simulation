from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world = LaunchConfiguration('world')
    debug = LaunchConfiguration('debug')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('my_simulation'),
                'world', 'maquette_world.world'
            )
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(
                get_package_share_directory('my_simulation'),
                'models'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ),
            launch_arguments={
                'world': world,
                'debug': debug,
                'gui': gui,
                'paused': pause,
                'use_sim_time': 'true'
            }.items()
        )
    ])