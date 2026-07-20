from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set Open3D environment variables
    # os.environ['Open3D_DIR'] = '/home/jajayu/open3d_install/lib/cmake/Open3D'
    # os.environ['LD_LIBRARY_PATH'] = '/home/jajayu/open3d_install/lib:' + os.environ['LD_LIBRARY_PATH']
    octomap_server_launch_file = os.path.join(
            FindPackageShare('octomap_server2').find('octomap_server2'), 'launch', 'octomap_server_launch.py'
        )


    # ============== Set Calibration Files =============
    

    # Define nodes to launch

        # Define remappings
    # remappings = [
    #     ('/camera/camera/color/image_raw', '/camera/image_raw'),  # Remap image topic
    #     ('/camera/camera/depth/color/points', '/camera/points')  # Remap points topic
    # ]
    return LaunchDescription([
        Node(
            package='my_robot_cam',
            executable='cam_cloudrate_transformer',
            name='cam_cloudrate_transformer_node',
            output='screen'#, 
            # remappings=[
            #     ('/camera/camera/color/image_raw', '/camera/image_raw'),  # Remap image topic
            #     ('/camera/camera/depth/color/points', '/camera/points')  # Remap points topic
            # ]
        ),
        Node(
            package='my_robot_nbv',
            executable='nbv_tom_detect',
            name='nbv_tom_detect_node',
            output='screen'
        ),
        Node(
            package='my_robot_nbv',
            executable='nbv_tompcd_filter',
            name='nbv_tompcd_filter_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            octomap_server_launch_file,
            launch_arguments={
                'resolution': '0.004', #0.03: will have space... but is good for now , but 0.04 is better 0.001: too small, leaving too much space# 0.01(for gazebo mode1) --> 0.001 (realsense real tomato size)
                'frame_id': 'base', #remember odom(for gazebo mode1,2) --> base_link(realsense)
                'input_cloud_topic': '/nbv/tompcd_ICP' #/cam/cloudrate_transformer
            }.items()
        ),
        Node(
            package='my_robot_nbv_cmake',
            executable='BestViewModel',
            name='best_view_model_node',
            output='screen'
        ),
        # LogInfo for any command output if needed
        LogInfo(
            condition=None,  # Replace with actual condition if required
            msg="ROS 2 launch started"
        )
    ])