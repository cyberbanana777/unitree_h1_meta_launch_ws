import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Получаем пути к launch-файлам других пакетов
    pkg1_launch_dir = os.path.join(get_package_share_directory('completed_scripts_control'), 'launch')
    pkg2_launch_dir = os.path.join(get_package_share_directory('completed_scripts_teleoperation'), 'launch')

    # Включаем первый launch-файл
    launch_file1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg1_launch_dir, 'control_H1_with_hands_launch.py'))
    )

    # Включаем второй launch-файл
    launch_file2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg2_launch_dir, 'teleoperation_launch.py'))
    )

    return LaunchDescription([
        launch_file1,
        launch_file2,
        # Можно добавить другие launch-файлы или ноды
    ])