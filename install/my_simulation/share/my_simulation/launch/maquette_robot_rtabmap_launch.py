from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	# Gazebo launch arguments
	world = LaunchConfiguration('world')
	debug = LaunchConfiguration('debug')
	gui = LaunchConfiguration('gui')
	pause = LaunchConfiguration('pause')
	model = LaunchConfiguration('model')

	# RTABMap launch arguments
	use_sim_time = LaunchConfiguration('use_sim_time')
	deskewing = LaunchConfiguration('deskewing')
	qos = LaunchConfiguration('qos')

	## ROS 2 Robot import ##

	pkg_share = FindPackageShare(package='my_simulation').find('my_simulation')
	default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')

	robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, {'robot_description': Command(['xacro ', model]), 'use_sim_time': True}]
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
			'-entity', 'my_first_robot', 
			'-topic', 'robot_description',
			'-x', '2.0',
			'-y', '-2.5',
			'-z', '0.0'
		],
		output='screen'
	)

	########################

	return LaunchDescription([
		########### Gazebo
		DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
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
        ),

        ## ROS 2 Robot import ##
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        ##ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,

		######### RTABMap
		DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
		DeclareLaunchArgument('deskewing', default_value='false', description='Enable lidar deskewing'),
		DeclareLaunchArgument('qos', default_value='2', description='QoS profile depth for lidar scan'),

		# TimerAction de 3 segundos para os nós do RTABMap
		TimerAction(
			period=3.0,
			actions=[
				Node(
					package='rtabmap_odom', executable='icp_odometry', output='screen',
					parameters=[{
						'frame_id':'lidar_link',
						'odom_frame_id':'odom',
						'map_frame_id':'map',
						'wait_for_transform':0.2,
						'expected_update_rate':15.0,
						'deskewing':deskewing,
						'use_sim_time':use_sim_time,
						'subscribe_rgb':False,
						'subscribe_depth':False,
						'subscribe_scan':True,
						'qos_scan': qos,
						'Icp/PointToPlane': 'true',
						'Icp/Iterations': '10',
						'Icp/VoxelSize': '0.1',
						'Icp/Epsilon': '0.001',
						'Icp/PointToPlaneK': '20',
						'Icp/PointToPlaneRadius': '0',
						'Icp/MaxTranslation': '2',
						'Icp/MaxCorrespondenceDistance': '1',
						'Icp/Strategy': '1',
						'Icp/OutlierRatio': '0.7',
						'Icp/CorrespondenceRatio': '0.01',
						'Odom/ScanKeyFrameThr': '0.4',
						'OdomF2M/ScanSubtractRadius': '0.1',
						'OdomF2M/ScanMaxSize': '15000',
						'OdomF2M/BundleAdjustment': 'false'
					}],
					remappings=[
						('scan_cloud', '/lidar/out')
					]
				),
				Node(
					package='rtabmap_slam', executable='rtabmap', output='screen',
					parameters=[{
						'frame_id':'lidar_link',
						'subscribe_depth':False,
						'subscribe_rgb':False,
						'subscribe_scan_cloud':True,
						'approx_sync':False,
						'wait_for_transform':0.2,
						'use_sim_time':use_sim_time,
						'RGBD/ProximityMaxGraphDepth': '0',
						'RGBD/ProximityPathMaxNeighbors': '1',
						'RGBD/AngularUpdate': '0.05',
						'RGBD/LinearUpdate': '0.05',
						'RGBD/CreateOccupancyGrid': 'false',
						'Mem/NotLinkedNodesKept': 'false',
						'Mem/STMSize': '30',
						'Mem/LaserScanNormalK': '20',
						'Reg/Strategy': '1',
						'Icp/VoxelSize': '0.1',
						'Icp/PointToPlaneK': '20',
						'Icp/PointToPlaneRadius': '0',
						'Icp/PointToPlane': 'true',
						'Icp/Iterations': '10',
						'Icp/Epsilon': '0.001',
						'Icp/MaxTranslation': '3',
						'Icp/MaxCorrespondenceDistance': '1',
						'Icp/Strategy': '1',
						'Icp/OutlierRatio': '0.7',
						'Icp/CorrespondenceRatio': '0.2'
					}],
					remappings=[
						('scan_cloud', '/lidar/out')
					],
					arguments=['-d']
				),
				Node(
					package='rtabmap_viz', executable='rtabmap_viz', output='screen',
					parameters=[{
						'frame_id':'lidar_link',
						'odom_frame_id':'odom',
						'map_frame_id':'map',
						'subscribe_odom_info':True,
						'subscribe_scan_cloud':True,
						'approx_sync':False,
						'use_sim_time':use_sim_time,
					}],
					remappings=[
						('scan_cloud', '/lidar/out')
					]
				),
			]
		),
	])
