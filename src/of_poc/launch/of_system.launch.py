import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'of_poc'

def generate_launch_description():
    # Define parameters or arguments for each node
    node1_params = {
        'use_sim_time': True,
        # 'param2': 'value2'
    }
    
    node2_params = {
        'use_sim_time': True,
        # 'paramB': 'valueB'
    }
    lucas_config = os.path.join(
    get_package_share_directory(PACKAGE_NAME),
    'config',
    'of_lucas.yaml'
    )
        
    # Create nodes
    node1 = Node(
        package=PACKAGE_NAME,
        executable='of_lucas_node',
        name='of_lucas_node',
        namespace='',
        parameters=[lucas_config],
        # remappings=[('/original_topic', '/remapped_topic')],
        output='screen',
    )

    node2 = Node(
        package=PACKAGE_NAME,
        executable='tf_handler',
        name='tf_handler',
        namespace='',
        parameters=[node2_params],
        output='screen'
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(node1)
    ld.add_action(node2)
    
    return ld
