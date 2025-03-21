#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="roomba_nav",
            executable="sensors_node",
            name="sensors_node",
            output="screen"
        ),
        Node(
            package="roomba_nav",
            executable="cmd_vel_bridge",
            name="cmd_vel_bridge",
            output="screen"
        ),
        Node(
            package="roomba_nav",
            executable="navigation_node",
            name="navigation_node",
            output="screen"
        )
    ])
