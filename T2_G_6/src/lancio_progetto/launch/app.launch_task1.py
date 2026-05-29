#!/usr/bin/env python3 

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    nodo1 = Node(package="aruco_pkg", executable="aruco_spegnimento")
    
    nodo2 = Node(package='move_head_action', executable='action_server')
    
    nodo3 = Node(package='move_head_action', executable='action_client')

    ld.add_action(nodo1)
    ld.add_action(nodo2)    
    ld.add_action(nodo3)

    return ld
