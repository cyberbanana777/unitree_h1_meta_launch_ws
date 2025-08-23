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
    # Получаем значение параметра mode
    mode = LaunchConfiguration('mode').perform(context)
    
    # Получаем пути к директориям с launch-файлами
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_control'),
        'launch'
    )
    pkg2_launch_dir = os.path.join(
        get_package_share_directory('completed_scripts_teleoperation'),
        'launch'
    )
    
    # Выбираем нужный control launch-файл в зависимости от режима
    if mode == 'with_hands':
        control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg1_launch_dir, 'control_H1_with_hands_launch.py'
                )
            )
        )
    elif mode == 'without_hands':
        control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg1_launch_dir, 'control_H1_without_hands_launch.py'
                )
            )
        )
    else:
        raise ValueError(f"Unknown mode: {mode}. Use 'with_hands' or 'without_hands'")
    
    # Teleoperation launch (общий для обоих режимов)
    teleoperation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg2_launch_dir, 'teleoperation_launch.py'
            )
        )
    )
    
    return [control_launch, teleoperation_launch]

def generate_launch_description():
    # Объявляем параметр для выбора режима
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='with_hands',
        description='Режим работы: with_hands или without_hands.',
        choices=['with_hands', 'without_hands']
    )
    
    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_setup)
    ])