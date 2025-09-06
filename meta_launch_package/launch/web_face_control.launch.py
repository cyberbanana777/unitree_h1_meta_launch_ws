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
    # Get value arg mode
    mode = LaunchConfiguration('mode').perform(context)
    
    # Get paths to directory with launch-files
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_control'),
        'launch'
    )
    pkg2_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_point-by-point_programming'),
        'launch'
    )
    
    # Node parameters
    common_params_control = {
        'mode': mode, 
        'target_topic': LaunchConfiguration('target_topic'),
        'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
        'target_action': LaunchConfiguration('target_action'),
    }

    button_params = {
        'mode' : mode,
    }

    # Select needed control launch-file depending on the mode
    if mode == 'with_hands' or mode == 'without_hands':
        control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg1_launch_dir, 'control_h1_base.launch.py'
                )
            ),
            launch_arguments=common_params_control.items()
        )
        web_face_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg2_launch_dir, 'control_by_buttons.launch.py'
                )
            ),
            launch_arguments=button_params.items()
        )
    else:
        raise ValueError(f"Unknown mode: {mode}. Use 'with_hands' or 'without_hands'")
    
    
    return [control_launch, web_face_launch]


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='with_hands',
        description='Режим работы: with_hands или without_hands',
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
        default_value='0.5',
        description='Maximum joint velocity.',
    )

    target_action_arg = DeclareLaunchArgument(
        'target_action',
        default_value='other',
        description='Target action for control commands.',
        choices=['other', 'teleoperation'],
    )
    
    return LaunchDescription([
        mode_arg,
        target_topic_arg,
        max_joint_velocity_arg,
        target_action_arg,

        OpaqueFunction(function=launch_setup)
    ])
