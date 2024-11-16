import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.actions import RegisterEventHandler,AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os

def generate_launch_description():
    #define empty launch description
    ld = launch.LaunchDescription()


    #define launch configuration parameter
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    robot_xacro = LaunchConfiguration("robot_xacro")
    world_name = LaunchConfiguration("world_name")
    xacro_pkg = LaunchConfiguration("xacro_pkg")
    
    control_cfg_file_name = LaunchConfiguration("control_cfg_file_name")
    log_level = LaunchConfiguration("log_level")
    xacro_pkg = LaunchConfiguration("xacro_pkg")
    use_lidar_3D = LaunchConfiguration("use_lidar_3D")
    control_cfg_pkg = LaunchConfiguration("control_config_pkg")
    #define launch arguments
    gz_verbosity_arg = DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4). Default is 3",
            )

    robot_xacro_arg = DeclareLaunchArgument(
                "robot_xacro",
                default_value="omnicar.xacro",
                description="Xacro File describe the spawing robot",
            )
    
    world_name_arg = DeclareLaunchArgument(
                "world_name",
                default_value="empty.sdf",
                description="World Name ",
            )

    xacro_pkg_arg = DeclareLaunchArgument(
                "xacro_pkg",
                default_value="mulinex_description",
                description="Xacro Package Name contains a installed urdf folder with xacro files"    
            )

    control_cfg_pkg_arg = DeclareLaunchArgument(
                "control_config_pkg",
                default_value="mulinex_ignition",
                description="Controller configuration file Package"    
            )

    control_cfg_file_name_arg = DeclareLaunchArgument(
                "control_cfg_file_name",
                default_value="omnicar.yaml",
                description="YAML File describe the robot controllers"    
            )

    log_lev_arg = DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            )

    use_lidar_3D_arg = DeclareLaunchArgument(
                "use_lidar_3D",
                default_value="true",
                description="Set 'true' to use the lidar 3D, as Velodyne, or lidar 2D, as RPLidarA2",
            )
    
    # get installed folder path 
    package_share_path = FindPackageShare('mulinex_ignition')
    

    # get configuration file path 
    xacro_file_path = PathJoinSubstitution(
        [
            FindPackageShare(xacro_pkg),
            'urdf', robot_xacro
        ]
    )
    # get controller configuration file path
    conf_file_path = PathJoinSubstitution(
        [
            package_share_path,
            'config', control_cfg_file_name
        ]
    )
    # get models file 
    models_path = PathJoinSubstitution(
        [
            package_share_path,
            'models'
        ]
    )

    world_path =  PathJoinSubstitution(
        [
            package_share_path,
            'world',
            world_name
        ]
    )

    #collect Environment variable path 
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')])}
    # set_gazebo_model_path_env = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_path)
    # define the simulator action 
    exec = 'ruby $(which gz) sim'
    sim = ExecuteProcess(
            cmd=[ exec, world_path,  '-r', '-v', gz_verbosity],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=True,
        )
    #define the bridge action
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output="screen",
    )

    # define robot state publisher node to provide urdf description 
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(
                ["xacro ", xacro_file_path,
                " use_gazebo:=true yaml_file:=",
                conf_file_path," lidar_3D:=",use_lidar_3D]
                )}
        ],
    )
    
    # define the spawner node 
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "mobile_robot",
            "-topic",
            "robot_description",
            "-z",
            "1.0",
            "-x",
            "-2.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": True}],
    )

    omni_control_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_control',
         '--param-file',
            conf_file_path],
    )
    
    # add arguments
    ld.add_action(gz_verbosity_arg)
    ld.add_action(robot_xacro_arg)
    ld.add_action(world_name_arg)
    ld.add_action(xacro_pkg_arg)
    ld.add_action(log_lev_arg)
    ld.add_action(control_cfg_pkg_arg)
    ld.add_action(control_cfg_file_name_arg)
    ld.add_action(use_lidar_3D_arg)

    #det env variable to export custom models
    ld.add_action(
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=models_path,
        )
    )

    # add action 
    ld.add_action(sim)
    ld.add_action(bridge)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)

    # load controller after spawn entity
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[omni_control_spawner],
            )
        )
    )
    return ld