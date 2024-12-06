# mulinex_ignition
it is a ros2 packege to manage the gazebo frotress simulation managing the gazebo world creation and the robot spawning.
For now the unique robot supported is the omnicar, i.e. omni-directional mobile robot, equipped with both proprioceptive sensors, including and IMU sensor and all the actuated joint state, and exteroceptive sensors, including a 3D lidar, Velodyne VLP16 like, and a deep camera, intel realsense D435 like.

## Launch Configuration Parameters

<ul>
    <li>gz_verbosity: Verbosity level for Ignition Gazebo (0~4). Default is 3</li>
    <li>log_level: The level of logging that is applied to all ROS 2 nodes launched by this script. Default is warn</li>
    <li>xacro_pkg: Xacro Package Name contains a installed urdf folder with xacro files. Default is  mulinex_description</li>
    <li>robot_xacro: Xacro File describe the spawing robot cointain in xacro_pkg package. Default is omnicar.xacro</li>
    <li>control_config_pkg: Controller configuration file Package. Default is mulinex_ignition</li>
    <li>control_cfg_file_name: YAML File describe the robot controllers. Default is omnicar.yaml</li>
    <li>joystic_teleop: Set 'true' to use the omni mulinex joystic node to teleop the robot. Default is true</li>
    <li>pub_tf_gt: Publish ground truth odometry tf, Pose and Path message for RViz2 visualization purpose. Default is false</li>
    <li>robot_namespace: tf and topic robot namespace, usefull for multi robot control purpose. Default is "omnicar/"</li>
    <li>world_name: world sdf file name, it must be present into mulinex_ignition package. Now empty.sdf and turtle_arena.sdf are supported</li>
</ul>

## Launch file 

the launch file to start the simulation is ` gz_harmonic_sim_w_rbt.launch.py `.