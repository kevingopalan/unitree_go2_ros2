import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = "base_link"

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="5.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )
    
    # Description nodes and parameters
    robot_description = {"robot_description": Command(["xacro ", LaunchConfiguration("unitree_go2_description_path")])}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # CHAMP controller nodes
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": base_frame},
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": base_frame},
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/urdf_viewer.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    # === SEPARATE BRIDGES FOR STABILITY ===

    # Clock Bridge (CRITICAL - Start First)
    unitree_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='unitree_clock_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'qos_overrides./clock.reliability': 'reliable'},
            {'qos_overrides./clock.durability': 'transient_local'},
        ],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # Camera Bridge (Delayed)
    unitree_camera_bridge = TimerAction(
        period=3.0,  # Wait for clock
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='unitree_camera_bridge',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'qos_overrides./rgb_image.reliability': 'best_effort'},
                    {'qos_overrides./rgb_image.durability': 'volatile'},
                    {'qos_overrides./rgb_image.history': 'keep_last'},
                    {'qos_overrides./rgb_image.depth': '1'},
                ],
                arguments=[
                    '/rgb_image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
            )
        ]
    )

    # LiDAR Bridge (Delayed)
    unitree_lidar_bridge = TimerAction(
        period=5.0,  # Wait for camera
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='unitree_lidar_bridge',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'qos_overrides./velodyne_points/points.reliability': 'best_effort'},
                    {'qos_overrides./unitree_lidar/points.reliability': 'best_effort'},
                    {'qos_overrides./velodyne_points/points.durability': 'volatile'},
                    {'qos_overrides./unitree_lidar/points.durability': 'volatile'},
                ],
                arguments=[
                    '/velodyne_points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    '/unitree_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/unitree_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ],
            )
        ]
    )

    # Sensor Bridge (IMU, Joint States, etc.) - Delayed
    unitree_sensor_bridge = TimerAction(
        period=7.0,  # Wait for LiDAR
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='unitree_sensor_bridge',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'qos_overrides./imu/data.reliability': 'best_effort'},
                    {'qos_overrides./joint_states.reliability': 'best_effort'},
                    {'qos_overrides./odom.reliability': 'best_effort'},
                ],
                arguments=[
                    '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                ],
            )
        ]
    )

    # Command Bridge (Control Commands) - Delayed
    unitree_command_bridge = TimerAction(
        period=9.0,  # Wait for sensors
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='unitree_command_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
                ],
            )
        ]
    )
    
    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path, 
            
            # Robot nodes (NO Gazebo simulation or spawning)
            robot_state_publisher_node,
            
            # CHAMP controller nodes
            quadruped_controller_node,
            state_estimator_node,
            
            # EKF nodes
            base_to_footprint_ekf,
            footprint_to_odom_ekf,
            
            # SEPARATE BRIDGES (STAGGERED TIMING)
            unitree_clock_bridge,        # 1. Clock first
            unitree_camera_bridge,       # 2. Camera (3s delay)
            unitree_lidar_bridge,        # 3. LiDAR (5s delay)  
            unitree_sensor_bridge,       # 4. Sensors (7s delay)
            unitree_command_bridge,      # 5. Commands (9s delay)
            
            # Visualization (only if rviz flag is set)
            rviz2,
        ]
    )