#!/usr/bin/env python3
"""
braccio_gazebo_moveit.launch.py
Launches:
  1. Gazebo Classic with braccio_world.world
  2. robot_state_publisher (URDF xacro processed)
  3. spawn_entity (spawns Braccio into Gazebo)
  4. ros2_control controller_manager + controller spawners
  5. MoveIt2 move_group node (SRDF, kinematics, moveit_controllers)
  6. RViz2 with MoveIt2 plugin

Usage:
  ros2 launch robot_control braccio_gazebo_moveit.launch.py
  ros2 launch robot_control braccio_gazebo_moveit.launch.py use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('robot_control')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # ── Arguments ──────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'braccio_world.world'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    # ── Process URDF ───────────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg, 'urdf', 'braccio.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ── MoveIt2 config files ────────────────────────────────────────────────────
    srdf_file = os.path.join(pkg, 'config', 'braccio.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = os.path.join(pkg, 'config', 'kinematics.yaml')
    joint_limits_yaml = os.path.join(pkg, 'config', 'joint_limits.yaml')
    moveit_controllers_yaml = os.path.join(pkg, 'config', 'moveit_controllers.yaml')
    ros2_controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    # ── 1. Gazebo ───────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'paused': LaunchConfiguration('paused'),
        }.items())

    # ── 2. robot_state_publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])

    # ── 3. Spawn robot in Gazebo ────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'braccio',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen')

    # ── 4. controller_manager (ros2_control) ────────────────────────────────────
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_yaml,
                    {'use_sim_time': use_sim_time}],
        output='screen')

    # Spawn controllers after controller_manager starts
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    # Delay arm/gripper controllers until joint_state_broadcaster is active
    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner]))

    # ── 5. MoveIt2 move_group ───────────────────────────────────────────────────
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time},
            kinematics_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': (
                    'default_planner_request_adapters/AddTimeOptimalParameterization '
                    'default_planner_request_adapters/ResolveConstraintFrames '
                    'default_planner_request_adapters/FixWorkspaceBounds '
                    'default_planner_request_adapters/FixStartStateBounds '
                    'default_planner_request_adapters/FixStartStateCollision '
                    'default_planner_request_adapters/FixStartStatePathConstraints'
                ),
                'start_state_max_bounds_error': 0.1,
                'publish_monitored_planning_scene': True,
                'capabilities': '',
                'disable_capabilities': '',
                'monitor_dynamics': False,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
            },
        ])

    # ── 6. RViz2 with MoveIt2 plugin ────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    {'use_sim_time': use_sim_time}],
        condition=None)   # always launch; pass use_rviz:=false to skip manually

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        paused_arg,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_arm,
        # Delay MoveIt2 until controllers are running
        TimerAction(period=5.0, actions=[move_group]),
        TimerAction(period=6.0, actions=[rviz]),
    ])
