#!/usr/bin/env python3
"""
Launch maestro de simulación Clearpath

- Robot: spawn aleatorio (x, y, yaw)
- Persona: spawn aleatorio con restricciones:
    * No aparece en Cuarto_2
    * No aparece en la misma ubicación del robot
- Persona creada después de 10 segundos
- SLAM + Nav2 + YOLO + nodos personalizados
- RViz

Autor: Jorman Castro Rivera
"""

import random
import math
from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # UBICACIONES DISPONIBLES
    # ============================================================
    locations = [
        {'name': 'Cuarto_1',   'x':  4.718846099643073,  'y': -1.3584528743291555},
        {'name': 'Cuarto_2',   'x':  6.189030618862003,  'y':  3.9954165849874137},
        {'name': 'Cuarto_3',   'x': 14.287490629863644,  'y': -0.7110321294290173},
        {'name': 'Sala_2',     'x': 12.845954252701455,  'y':  2.637096295983294},
        {'name': 'Sala_cajas', 'x': 20.792694362182534,  'y': -3.023138266068986},
        {'name': 'Cuarto_4',   'x': 25.16951324784604,   'y': -4.552113969421629},
        {'name': 'Comedor',    'x': -9.099209488539879,  'y': -1.6634411022393856}
    ]

    # ============================================================
    # ROBOT (aleatorio)
    # ============================================================
    robot_pose = random.choice(locations)
    yaw = random.uniform(0.0, 2.0 * math.pi)

    # ============================================================
    # PERSONA (con restricciones)
    # ============================================================
    person_locations = [
        loc for loc in locations
        if loc['name'] != 'Cuarto_2'
        and loc['name'] != robot_pose['name']
    ]

    person_pose = random.choice(person_locations)

    print("\n==============================")
    print("🤖 ROBOT")
    print(f"x={robot_pose['x']:.3f}  y={robot_pose['y']:.3f}")
    print(f"yaw={yaw:.3f} rad")
    print("🧍 PERSONA")
    print(f"x={person_pose['x']:.3f}  y={person_pose['y']:.3f}")
    print("==============================\n")

    # ============================================================
    # PATHS
    # ============================================================
    clearpath_gz = FindPackageShare('clearpath_gz')
    nav2_demos = FindPackageShare('clearpath_nav2_demos')
    yolo_bringup = FindPackageShare('yolo_bringup')
    clearpath_viz = FindPackageShare('clearpath_viz')

    setup_path = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'clearpath/'
    ])

    person_sdf = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'person.sdf'
    ])

    # ============================================================
    # GAZEBO SIMULATION
    # ============================================================
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            clearpath_gz,
            '/launch/simulation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': 'office',
            'x': str(robot_pose['x']),
            'y': str(robot_pose['y']),
            'yaw': str(yaw)
        }.items()
    )

    # ============================================================
    # SPAWN PERSONA (después de 10s)
    # ============================================================
    spawn_person = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'office',
            '-name', 'person',
            '-file', person_sdf,
            '-x', str(person_pose['x']),
            '-y', str(person_pose['y']),
            '-z', '0.0'
        ],
        output='screen'
    )

    # ============================================================
    # SLAM
    # ============================================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_demos,
            '/launch/slam.launch.py'
        ]),
        launch_arguments={
            'setup_path': setup_path,
            'use_sim_time': 'true',
            'namespace': 'a200_0000'
        }.items()
    )

    # ============================================================
    # NAV2
    # ============================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_demos,
            '/launch/nav2.launch.py'
        ]),
        launch_arguments={
            'setup_path': setup_path,
            'use_sim_time': 'true',
            'namespace': 'a200_0000'
        }.items()
    )

    # ============================================================
    # YOLO
    # ============================================================
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            yolo_bringup,
            '/launch/yolo.launch.py'
        ])
    )

    # ============================================================
    # NODOS PROPIOS
    # ============================================================
    explorer_node = Node(
        package='custom_explorer',
        executable='explorer',
        output='screen'
    )

    person_goal_node = Node(
        package='person_goal_navigator',
        executable='person_goal_node',
        output='screen'
    )

    # ============================================================
    # RVIZ
    # ============================================================
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            clearpath_viz,
            '/launch/view_navigation.launch.py'
        ]),
        launch_arguments={
            'namespace': 'a200_0000',
            'use_sim_time': 'true'
        }.items()
    )
    # ============================================================
    # SECUENCIA DE ARRANQUE
    # ============================================================

    spawn_person_delay = TimerAction(
        period=30.0,
        actions=[spawn_person]
    )

    slam_delay = TimerAction(
        period=35.0,
        actions=[slam_launch]
    )

    nav2_delay = TimerAction(
        period=45.0,
        actions=[nav2_launch]
    )

    yolo_delay = TimerAction(
        period=65.0,
        actions=[yolo_launch]
    )

    explorer_delay = TimerAction(
        period=85.0,
        actions=[explorer_node]
    )

    person_goal_delay = TimerAction(
        period=105.0,
        actions=[person_goal_node]
    )

    viz_delay = TimerAction(
        period=120.0,
        actions=[viz_launch]
    )


    return LaunchDescription([
        simulation_launch,

        spawn_person_delay,
        slam_delay,
        nav2_delay,
        yolo_delay,
        explorer_delay,
        person_goal_delay,
        viz_delay
    ])
