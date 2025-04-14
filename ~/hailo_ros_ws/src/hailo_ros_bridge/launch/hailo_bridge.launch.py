from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    zmq_address_arg = DeclareLaunchArgument(
        'zmq_address',
        default_value='tcp://localhost:5555',
        description='ZeroMQ address to connect to'
    )
    
    # Create the subscriber node
    subscriber_node = Node(
        package='hailo_ros_bridge',
        executable='zmq_subscriber',
        name='hailo_zmq_subscriber',
        parameters=[{
            'zmq_address': LaunchConfiguration('zmq_address')
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        zmq_address_arg,
        subscriber_node
    ]) 