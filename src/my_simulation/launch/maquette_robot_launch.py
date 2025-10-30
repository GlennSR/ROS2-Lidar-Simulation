from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_simulation')

    # Paths
    default_robot_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    only_lidar_robot_path = os.path.join(pkg_share, 'urdf', 'only_lidar', 'only_lidar.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    default_world_path = os.path.join(pkg_share, 'world', 'maquette_vf_world.sdf')

    # Launch configurations
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')
    only_lidar = LaunchConfiguration('only_lidar')
    rviz_arg = LaunchConfiguration('rvizconfig')

    # Robot Model Parameters
    # Spawn pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    # Lidar inclination
    lidar_inclination = LaunchConfiguration('lidar_angle')


    # Set Gazebo model path
    set_gz_model_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=os.pathsep.join([
        os.path.join(pkg_share, 'models'),
        ])
    )

    robot_state_publisher_default = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'robot_description': Command([
            'xacro ', default_robot_path,
            ' lidar_inclination:=', lidar_inclination
        ])
    }],
    condition=UnlessCondition(only_lidar)
)

    robot_state_publisher_lidar = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                'xacro ', only_lidar_robot_path,
                ' lidar_inclination:=', lidar_inclination
            ])
        }],
        condition=IfCondition(only_lidar)
    )

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(pkg_share,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )

    

    # Spawn the robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_core',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z
        ],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_arg],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('only_lidar', default_value='False', description='Spawn only lidar without robot'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('pause', default_value='false', description='Pause simulation on start'),
        DeclareLaunchArgument('world', default_value=default_world_path, description='Path to the Gazebo world file'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='Path to RViz config'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial X position of the robot'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position of the robot'),
        DeclareLaunchArgument('z', default_value='0.0', description='Initial Z position of the robot'),
        DeclareLaunchArgument('lidar_angle', default_value=str(3.1415/4), description='lidar inclination angle'),

        set_gz_model_path,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        robot_state_publisher_default,
        robot_state_publisher_lidar,   
        spawn_entity,
        rviz_node
    ])
