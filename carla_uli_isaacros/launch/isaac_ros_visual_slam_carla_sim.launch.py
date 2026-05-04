import launch
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Visual SLAM Node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('visual_slam/image_0', '/carla/hero/rgb_left/image_synced'),
            ('visual_slam/camera_info_0', '/carla/hero/rgb_left/camera_info_fixed_synced'),
            ('visual_slam/image_1', '/carla/hero/rgb_right/image_synced'),
            ('visual_slam/camera_info_1', '/carla/hero/rgb_right/camera_info_fixed_synced')
        ],
        parameters=[{
            'use_sim_time': True,
            'enable_image_denoising': True,
            'rectified_images': False,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
        }]
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    # TF publishers
    static_tf_rgb_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_rgb_left',
        arguments=['1.5', '0.0', '1.2', '0', '0', '0', 'base_link', 'rgb_left']
    )

    static_tf_rgb_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_rgb_right',
        arguments=['1.5', '0.3', '1.2', '0', '0', '0', 'base_link', 'rgb_right']
    )

    static_tf_map_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    static_tf_base_to_hero = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_hero',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'hero']
    )

    
    # Clock-synchronized camera bridge
    synched_fixed_camea_node = Node(
        package='carla_uli_isaacros',
        executable='clock_synced_stereo_camera_node',  # Make sure this is installed in your package
        name='clock_synced_stereo_camera_node',
        parameters=[{'use_sim_time': True}]
    )

    # # Image rectification for stereo
    # image_proc_left = Node(
    #     package='image_proc',
    #     executable='image_proc',
    #     name='image_proc_left',
    #     remappings=[
    #         ('image_raw', '/carla/hero/rgb_left/image_synced'),
    #         ('camera_info', '/carla/hero/rgb_left/camera_info_fixed_synced'),
    #         ('image', '/carla/hero/rgb_left/image_rect_color')
    #     ],
    #     parameters=[{'use_sim_time': True}]
    # )

    # image_proc_right = Node(
    #     package='image_proc',
    #     executable='image_proc',
    #     name='image_proc_right',
    #     remappings=[
    #         ('image_raw', '/carla/hero/rgb_right/image_synced'),
    #         ('camera_info', '/carla/hero/rgb_right/camera_info_fixed_synced'),
    #         ('image', '/carla/hero/rgb_right/image_rect_color')
    #     ],
    #     parameters=[{'use_sim_time': True}]
    # )

    return launch.LaunchDescription([
        static_tf_rgb_left,
        static_tf_rgb_right,
        static_tf_map_base,
        static_tf_base_to_hero,
        synched_fixed_camea_node,
        # image_proc_left,
        # image_proc_right,
        visual_slam_container,
    ])
