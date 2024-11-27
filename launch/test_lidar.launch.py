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


    
    #define the bridge action
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
          

            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/rgl_lidar@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
        output="screen",
    )
    tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link velodyne'.split(' '),
            output='screen'
            )
    # define robot state publisher node to provide urdf description 

    
    # define the spawner node 
    # spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=[
    #         "-name",
    #         "mobile_robot",
    #         "-topic",
    #         "robot_description",
    #         "-z",
    #         "1.0",
    #         "-x",
    #         "-2.0",
    #         "--ros-args",
    #         "--log-level",
    #         log_level,
    #     ],
    #     parameters=[{"use_sim_time": True}],
    # )

    

    # ld.add_action(bridge)
    ld.add_action(tf)
    
    return ld