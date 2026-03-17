from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            '/home/theo/Individual-project/bags/bags/track_test2',      # <-- change to your bag path
            '--loop'
        ],
        output='screen'
    )

    # Republish raw → compressed
    image_republish = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=[
            'raw', 'compressed',
            '--ros-args',
            '-r', 'in:=/camera/image_raw',              # input topic
            '-r', 'out/compressed:=/camera/image_raw/compressed'  # output topic
        ],
        output='screen'
    )

    # Open rqt_image_view
    rqt_view = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
            '--ros-args',
            '-p', '_image_transport:=compressed'  # view compressed topic
        ],
        output='screen'
    )


    return LaunchDescription([
        bag_play,
        image_republish,
        rqt_view
    ])