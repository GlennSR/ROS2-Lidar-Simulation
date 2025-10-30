from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world = LaunchConfiguration('world')
    debug = LaunchConfiguration('debug')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')
    robot = LaunchConfiguration('robot')
    rviz_arg = LaunchConfiguration('rvizconfig')

    # Robot Model Parameters   
    # Spwan pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    # Lidar inclination
    lidar_inclination = LaunchConfiguration('lidar_angle')
    
    ## ROS 2 Robot import ##
    pkg_share = FindPackageShare(package='my_simulation').find('my_simulation')
    default_robot_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    only_lidar_robot_path = os.path.join(pkg_share, 'urdf/only_lidar/only_lidar.urdf.xacro')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_config.rviz')
    

    robot_state_publisher_node_default = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'use_sim_time': True,
        'robot_description': Command([
            'xacro ', default_robot_path,
            ' lidar_inclination:=', lidar_inclination])
    }],
    condition=UnlessCondition(LaunchConfiguration('only_lidar'))
    )

    robot_state_publisher_node_origin = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'use_sim_time': True,
        'robot_description': Command([
            'xacro ', only_lidar_robot_path,
            ' lidar_inclination:=', lidar_inclination])
    }],
    condition=IfCondition(LaunchConfiguration('only_lidar'))
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
        '-entity', 'robot_core', 
        '-topic', 'robot_description',
        '-x', x,
        '-y', y,
        '-z', z
        ],
        output='screen'
        )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_arg],
        parameters=[{'use_sim_time': True}]
    )

    ########################

    return LaunchDescription([
        DeclareLaunchArgument('only_lidar', description='Uses the other urdf to spawn only lidar, without robot, at origin', default_value='False'),
        DeclareLaunchArgument('debug', description='Enable debug mode', default_value='false'),
        DeclareLaunchArgument('gui', description='Enable Gazebo GUI', default_value='true'),
        DeclareLaunchArgument('pause', description='Pause simulation on start', default_value='false'),
        DeclareLaunchArgument(
            'world', 
            description='Full path to world model file to load',
            default_value=os.path.join(
                get_package_share_directory('my_simulation'),
                'world', 'maquette_vf_world.world'
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
        ),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='x', default_value='0.0', description='Initial X position of the robot'),
        DeclareLaunchArgument(name='y', default_value='0.0', description='Initial Y position of the robot'),
        DeclareLaunchArgument(name='z', default_value='0.0', description='Initial Z position of the robot'),
        DeclareLaunchArgument(name='lidar_angle', default_value=str(3.1415/4), description='Lidar inclination angle in degrees'),

        ## ROS 2 Robot import ##
        DeclareLaunchArgument(name='robot', default_value=default_robot_path,
                                            description='Absolute path to robot urdf file'),
        joint_state_publisher_node,
        robot_state_publisher_node_origin,
        robot_state_publisher_node_default,
        spawn_entity,
        rviz_node
    ])