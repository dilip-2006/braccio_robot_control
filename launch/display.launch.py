#!/usr/bin/env python3
"""
display.launch.py
=================
Launches robot_state_publisher + joint_state_publisher_gui + RViz2
for visualising and manually controlling the Braccio arm (no Gazebo).

Usage:
  ros2 launch robot_control display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('robot_control')

    # ── Process URDF (xacro) with package mappings ──────────────────────────
    urdf_file = os.path.join(pkg, 'urdf', 'braccio.urdf.xacro')
    doc = xacro.process_file(
        urdf_file,
        mappings={'ros2_controllers_config': os.path.join(pkg, 'config', 'ros2_controllers.yaml')}
    )
    robot_description_str = doc.toxml()

    rviz_config_file = os.path.join(pkg, 'config', 'braccio_display.rviz')

    # ── Launch Arguments ─────────────────────────────────────────────────────
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher_gui (set false if joint_states come from elsewhere)')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2')

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    from launch.actions import LogInfo
    
    # ── Nodes ────────────────────────────────────────────────────────────────
    
    welcome_msg = LogInfo(msg="""
=========================================================
  ____                      _                        
 | __ ) _ __ __ _  ___ ___ (_) ___                 
 |  _ \| '__/ _` |/ __/ __|| |/ _ \           
 | |_) | | | (_| | (_| (__ | | (_) |               
 |____/|_|  \__,_|\___\___||_|\___/               
                                                        
 Starting Manual Control Interface      
 By Dilip Kumar
=========================================================
""")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_str,
            'use_sim_time': False,
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': False,
        }],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        welcome_msg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ])
