import sys
import os
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def main():
    home = os.path.expanduser('~')
    map_yaml_path = os.path.join(home, 'ros2_ws/src/ras598_assignment_2/map.yaml')
    scout_script_path = os.path.join(home, 'ros2_ws/src/ras598_assignment_2/grading_scout.py')
    ld = LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': True}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]
        ),
        ExecuteProcess(
            cmd=['python3', scout_script_path],
            output='screen'
        ),
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    print("--- Starting ROS 2 Launch Service ---")
    return ls.run()

if __name__ == '__main__':
    sys.exit(main())
