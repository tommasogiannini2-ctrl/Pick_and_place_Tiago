#!/usr/bin/env python3 

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    # Nodi base
    nodo1 = Node(package="aruco_pkg", executable="aruco_spegnimento")
    nodo2 = Node(package='move_head_action', executable='action_server')
    nodo3 = Node(package='move_head_action', executable='action_client')

    # Processo per il braccio ritardato
    nodo_braccio = ExecuteProcess(
        cmd=['ros2', 'run', 'play_motion2', 'run_motion', 'unfold_arm', 'false', '30'],
        output='screen'
    )
    
    delayed_braccio = TimerAction(
        period=10.0,
        actions=[nodo_braccio]
    )

    # Nodo Coca-Cola
    nodo_cola = Node(
        package='pick_and_place', 
        executable='pick_place_action',
        output='screen'
    )
    
    delayed_cola = TimerAction(
        period=20.0,
        actions=[nodo_cola]
    )

    # Nodo Pringles 
    nodo_pringles = Node(
        package='pick_and_place', 
        executable='pick_place_pringles',
        output='screen'
    )

    evento_pringles = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=nodo_cola,
            on_exit=[nodo_pringles]
        )
    )
    
    
    nodo_saluto = ExecuteProcess(
        cmd=['ros2', 'run', 'play_motion2', 'run_motion', 'wave'],
        output='screen'
    )

    # Il saluto si attiva SOLO quando il nodo delle Pringles finisce (OnProcessExit)
    evento_saluto = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=nodo_pringles,
            on_exit=[nodo_saluto]
        )
    )

    # Aggiunta alla LaunchDescription
    ld.add_action(nodo1)
    ld.add_action(nodo2)    
    ld.add_action(nodo3)
    ld.add_action(delayed_braccio) 
    ld.add_action(delayed_cola) 
    ld.add_action(evento_pringles)
    ld.add_action(evento_saluto)

    return ld
