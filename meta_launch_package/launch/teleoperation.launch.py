# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get value arg 'mode'
    mode = LaunchConfiguration('mode').perform(context)

    # Node parameters
    common_params = {
        'mode': mode,
        'target_topic': LaunchConfiguration('target_topic'),
        'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
        'target_action': LaunchConfiguration('target_action'),
    }
    
    teleoperation_params = {
        'enable_metrics': LaunchConfiguration('enable_metrics'),
        'joints_to_check': LaunchConfiguration('joints_to_check'),
        'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
        'ip': LaunchConfiguration('ip'),
    }
    
    # Get paths to directory with launch-files
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_control'),
        'launch'
    )
    pkg2_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_teleoperation'),
        'launch'
    )
    
    # Select needed control launch-file depending on the mode
    if mode == 'with_hands' or mode == 'without_hands':
        control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg1_launch_dir, 'control_h1_base.launch.py'
                )
            ),
            launch_arguments=common_params.items()
        )
    else:
        raise ValueError(f"Unknown mode: {mode}. Use 'with_hands' or 'without_hands'")
    
    # Teleoperation launch (common for both modes)
    teleoperation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg2_launch_dir, 'teleoperation.launch.py'
            )
        ),
        launch_arguments=teleoperation_params.items()
    )
    
    return [control_launch, teleoperation_launch]


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='with_hands',
        description='Режим работы: with_hands или without_hands.',
        choices=['with_hands', 'without_hands']
    )

    target_topic_arg = DeclareLaunchArgument(
        'target_topic',
        default_value='arm_sdk',
        description='Topic for control commands.',
        choices=['arm_sdk', 'lowcmd'],
    )

    max_joint_velocity_arg = DeclareLaunchArgument(
        'max_joint_velocity',
        default_value='4.0',
        description='Maximum joint velocity.',
    )

    target_action_arg = DeclareLaunchArgument(
        'target_action',
        default_value='teleoperation',
        description='Target action for control commands.',
        choices=['other', 'teleoperation'],
    )

    metrics_arg = DeclareLaunchArgument(
        "enable_metrics",
        default_value="False",
        description="Enable metrics publishing.",
        choices=['True', 'False'],
    )

    joints_to_check_arg = DeclareLaunchArgument(
        "joints_to_check",
        default_value="12, 13, 14, 15, 31, 33",
        description="Joints to check. Valided joints must " \
        "be in range [0, 33], excluding 9.",
    )

    ip_arg = DeclareLaunchArgument(
        "ip",
        default_value="192.168.123.162",
        description="Ip address for reading data from the UKT",
    )


    
    return LaunchDescription([
        mode_arg,
        target_topic_arg,
        max_joint_velocity_arg,
        target_action_arg,
        metrics_arg,
        joints_to_check_arg,
        ip_arg,

        OpaqueFunction(function=launch_setup)
    ])