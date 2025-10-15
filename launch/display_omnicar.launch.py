import imp
import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition,UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #path for build mulinex12 urdf
    omnicar_robot_path = get_package_share_path("mulinex_description")
    omnicar_robot_path = os.path.join(omnicar_robot_path,"urdf", "omnicar.xacro") 
    # #path for rviz settings
    rviz_config_path = get_package_share_path("mulinex_description")
    rviz_config_path = os.path.join(rviz_config_path, "rviz", "config.rviz")

    #declaration argument of launch
    use_gui = DeclareLaunchArgument(
        name="use_gui",
        default_value="true",
        description="Value use to enable joint publisher with GUI")

    hunter_model = DeclareLaunchArgument(
        name="omnicar_model",
        default_value=str(omnicar_robot_path)
    )

    # rviz_arg = DeclareLaunchArgument(
    #     name="rviz_config",
    #     default_value= str(rviz_config_path),
    #     description="configuration of Rviz for plot"
    # )

    #use command to create a parameter with urdf of mulinex by xacro file
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("omnicar_model")]),
        value_type=str
    )

    #node declaration
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    robot_joint_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition= UnlessCondition(LaunchConfiguration('use_gui')) 
    )

    robot_joint_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition= IfCondition(LaunchConfiguration('use_gui')) 
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output= 'screen',
        arguments=['-d', rviz_config_path],
    )



    return LaunchDescription(
        [
            hunter_model,
            # rviz_arg,
            use_gui,
            rviz_node,
            robot_joint_pub,
            robot_joint_pub_gui,
            robot_state_pub


        ]
    )