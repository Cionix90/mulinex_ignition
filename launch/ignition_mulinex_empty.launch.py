from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler,OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import numpy as np
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world_name = LaunchConfiguration('world',default="-r empty.sdf")
    #Declare Launch



    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    world_name_arg = DeclareLaunchArgument(
            'world',
            default_value=world_name,
            description='If true, use simulated clock')

    #get world name

    world = PathJoinSubstitution(
                [FindPackageShare('gazebo_models_worlds_collection'),
                 'worlds', world_name]
            )

    # Get URDF via xacro
    hip_angle = 0.0
    knee_angle = 0.0

    default_dof    = (
            np.deg2rad(hip_angle),     
            -(np.deg2rad(hip_angle)),    
            -(np.deg2rad(hip_angle)),
            np.deg2rad(hip_angle),
            -np.deg2rad(knee_angle),   
            np.deg2rad(knee_angle),    
            np.deg2rad(knee_angle),
            -np.deg2rad(knee_angle),
    )

    joint_names=(
        'LF_HFE',   
        'LH_HFE',   
        'RF_HFE',
        'RH_HFE',
        'LF_KFE',   
        'LH_KFE',   
        'RF_KFE',
        'RH_KFE',
    )
    default_dict = dict(zip(joint_names, default_dof))

    #format is "NAME:=VALUE "
    default_joint_args = ""
    for key, value in default_dict.items():
        default_joint_args += key + ":=" + str(value) + " "

    # Get URDF via xacro
   
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('mulinex_description'),
                 'urdf', 'mulinex.xacro']
            ),
             " use_gazebo:=true ",
            default_joint_args
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('mulinex_ignition'),
            'config',
            'mulinex_mf.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'mulinex', '-allow_renaming', 'true', "-z" , "0.5"],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
   
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                launch_arguments={
                    "gz_args": world_name
                }.items()),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        use_sim_time_arg,
        world_name_arg
        
    ])