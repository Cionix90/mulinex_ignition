import imp
import os
from ament_index_python.packages import get_package_share_path
import yaml
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition,UnlessCondition
from launch.actions import OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def create_yaml(context: LaunchContext, robot_name,joystic_teleop):
    robot_name_str = context.perform_substitution(robot_name)
    
    # teleop_node = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     condition=IfCondition(joystic_teleop),
    #     parameters=[os.path.join(get_package_share_path("mulinex_ignition"),"config", "teleop_conf.yaml")],
    #     remappings=[
    #             ('/cmd_vel', "/omni_control/command")]
    # )
    teleop_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('omni_mulinex_joystic'),
                    'launch',
                    'start_joy_sim.launch.py'
                ])
            ]),
            condition=IfCondition(joystic_teleop)
        )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            ["imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
            # [robot_name,"gt_odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry"],
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            ["rgl_lidar@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"],
            # "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            # "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
            # "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
        ],
        output="screen",
        )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )  
    omni_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_control", "--controller-manager", "/controller_manager"],
        )  
    return [teleop_node,bridge,omni_control,joint_state_broadcaster]


def generate_launch_description():

    ld = LaunchDescription()
    
    log_level = LaunchConfiguration("log_level")
    robot_name = LaunchConfiguration("robot_name")
    spawn_pos_x = LaunchConfiguration("spawn_pos_x")
    spawn_pos_y = LaunchConfiguration("spawn_pos_y")
    spawn_pos_z = LaunchConfiguration("spawn_pos_z")
    joystic_teleop = LaunchConfiguration("joystic_teleop")

    omnicar_robot_path = get_package_share_path("mulinex_description")
    omnicar_robot_path = os.path.join(omnicar_robot_path,"urdf", "omnicar.xacro")

    omnicar_model = DeclareLaunchArgument(
        name="omnicar_urdf",
        default_value=str(omnicar_robot_path)
    )
    log_lev_arg = DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            )

    robot_name_arg = DeclareLaunchArgument(
                name="robot_name",
                default_value="",
                description="Robot Name, Usefull to spawn different robots in the same simulation",
            )
    spawn_pos_x_arg = DeclareLaunchArgument(
            name="spawn_pos_x",
            default_value="0.0",
            description="spawn x position",
        )
    spawn_pos_y_arg = DeclareLaunchArgument(
            name="spawn_pos_y",
            default_value="0.0",
            description="spawn y position",
        )
    spawn_pos_z_arg = DeclareLaunchArgument(
            name="spawn_pos_z",
            default_value="0.2",
            description="spawn z position",
        )
    joystic_teleop_arg = DeclareLaunchArgument(
            name="joystic_teleop",
            default_value="true",
            description="use joystic to teleoperate the robot ",
        )
    ld.add_action(omnicar_model)
    ld.add_action(log_lev_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(spawn_pos_x_arg)
    ld.add_action(spawn_pos_y_arg)
    ld.add_action(spawn_pos_z_arg)
    ld.add_action(joystic_teleop_arg)

    #use command to create a parameter with urdf of mulinex by xacro file
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("omnicar_urdf")]),
        value_type=str
    )

    #node declaration
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{'robot_description': robot_description}]
    )
    ld.add_action(robot_state_pub)

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=robot_name,
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-z",
            spawn_pos_z,
            "-x",
            spawn_pos_x,
            "-y",
            spawn_pos_y,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": True}],
    )
    ld.add_action(spawn_entity)

    ld.add_action(OpaqueFunction(
        function=create_yaml, args=[robot_name,joystic_teleop]
    ))

    joy_node = Node(
        package="joy",
        executable="joy_node",
        condition=IfCondition(joystic_teleop)
    )

    ld.add_action(joy_node)
    # teleop_node = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     condition=IfCondition(joystic_teleop),
    #     parameters=[os.path.join(get_package_share_path("scout_description"),"config", "teleop_conf.yaml")],
    #     remappings=[
    #             ('/cmd_vel', "/"+ +"/ackerman_control/reference")]
    # )
    # ld.add_action(teleop_node)
    return ld