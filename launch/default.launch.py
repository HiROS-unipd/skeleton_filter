from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package='hiros_skeleton_filter',
        executable='hiros_skeleton_filter',
        name='skeleton_filter',
        namespace='hiros',
        output='screen',
        parameters=[
            {'input_topic': '/input/topic'},
            {'output_topic': '/output/topic'},
            {'filter': 'butterworth'},
            {'butterworth_order': -1},
            {'sample_frequency': -1.},
            {'cutoff_frequency': 6.},
        ]
    )

    ld.add_action(node)
    return ld
