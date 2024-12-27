from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    driver_launch_path = os.path.join(
        get_package_share_directory('tello_driver'),  
        'launch',
        'tello_driver.launch.py' 
    )

    takeoff_process = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/takeoff', 'std_msgs/msg/Empty', '{}', '--once'],
        output='screen'
    )

    land_process = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/land', 'std_msgs/msg/Empty', '{}', '--once'],
        output='screen'
    )

    land_event_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[land_process])
    )

    return LaunchDescription([

        ################ Tello driver ################
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(driver_launch_path)
        # ),

        ################ Face tracking ################

        Node(
            package='face_tracking',  
            executable='face_tracker',  
        ),

        Node(
            package='webcam_bridge',
            executable='webcam_publisher',
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/tracked_faces'],
            output='screen'
        ),

        ################ PID controller ################

        Node(
            package='drone_pid_controller',  
            executable='pid_controller',  
        ),
        #Pid plots 
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot', '/cmd_vel/linear'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot', '/cmd_vel/angular'],
            output='screen'
        ),
        ################ Landing and takeoff ################
        takeoff_process,  
        land_event_handler,
    ])
