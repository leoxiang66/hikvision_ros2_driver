from launch import LaunchDescription
from launch_ros.actions import Node


params = [
    {'exposure_time': 15000}, 
    {'frequency': 20},
    {'sync_point': 10000000}, # wait until the next `sync_point`
]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikvision_ros2_driver',
            executable='image_publisher',
            name='hikvision_image_publisher',
            parameters= params,
            output='screen',  
        )
    ])