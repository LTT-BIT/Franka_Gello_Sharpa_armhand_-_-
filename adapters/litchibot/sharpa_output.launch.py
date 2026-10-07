"""Explicit future output bringup using the existing Sharpa driver only."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    raise RuntimeError('Deprecated parallel hardware launch disabled; use production Terminal 2 dry-run')
    if LaunchConfiguration('enable_output').perform(context).lower() != 'true':
        raise RuntimeError('Output disabled. This launch requires enable_output:=true explicitly.')
    root = Path(__file__).resolve().parents[2]
    return [
        Node(package='sharpa_driver', executable='sharpa_driver', name='litchibot_sharpa_driver',
            output='screen', parameters=[{
                'use_fake_hardware': False, 'enable_left': True, 'enable_right': True,
                'left_serial': LaunchConfiguration('left_serial'),
                'right_serial': LaunchConfiguration('right_serial'),
                'speed_coefficient': 0.3, 'current_coefficient': 0.6,
                'stop_on_command_timeout_s': 0.25,
            }]),
        ExecuteProcess(cmd=[sys.executable,'-m','adapters.litchibot.ros_relay','--enable-output'],
                       cwd=str(root),output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_output',default_value='false'),
        DeclareLaunchArgument('left_serial',default_value='C5549534C555'),
        DeclareLaunchArgument('right_serial',default_value='CC559039CC54'),
        OpaqueFunction(function=start),
    ])
