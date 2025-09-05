# nav_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 텅 빈 지도의 YAML 파일 경로
    map_file_path = os.path.join(
        get_package_share_directory('fast_lio'), # 실제 패키지 이름으로 변경
        'maps',
        'my_map.yaml'
    )

    # Map Server를 관리할 Lifecycle Manager 노드
    lifecycle_nodes = ['map_server']
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    # 텅 빈 지도를 로드하고 /map 토픽을 발행할 Map Server 노드
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename': map_file_path}]
    )
    # 2. map -> odom TF 발행 노드
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 3. Nav2 Launch 파일 포함
    # 방금 만든 nav2_params.yaml 파일의 경로를 지정합니다.
    nav2_params_path = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'nav2_params_mppi.yaml')
        
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav2_params_path}.items()
    )
    
    # # 4. RViz 실행 노드 (선택 사항)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')]
    # )

     # 새로운 Remap 노드 추가
    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['/cmd_vel', '/move_base/cmd_vel']
    )

    return LaunchDescription([
        lifecycle_manager_node,
        map_server_node,
        map_to_odom_node,
        nav2_launch,
        cmd_vel_relay_node,
        # rviz_node
    ])