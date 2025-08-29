from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='whycon',
            name='whycon',
            namespace='whycon',
            executable='whycon',
            output='screen',
            parameters=[{
                'targets': 1,
                'name': 'whycon',
                'outer_diameter': 0.038,
                'inner_diameter': 0.014
            }],
            remappings=[
                ('image_raw', '/camera/image_raw')
            ]
        ),
        Node(
            package='image_view',
            executable='image_view',
            namespace='whycon_display',
            name='image_view',
            output='screen',
            remappings=[
                ('image', '/whycon/image_out')
            ]
        )
    ])
