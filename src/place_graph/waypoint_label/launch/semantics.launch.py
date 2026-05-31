import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _locate_robot_yaml():
    env = os.environ.get('TYPEGO_CONFIG')
    if env and os.path.isfile(env):
        return env
    try:
        share = get_package_share_directory('typego_config')
    except Exception:
        return None
    candidate = os.path.join(share, 'config', 'robot.yaml')
    return candidate if os.path.isfile(candidate) else None


def _edge_service_endpoint():
    """Build the qwenvl `/process` URL from robot.yaml's network.edge_service.

    Read with plain PyYAML — ros2 launch runs under /usr/bin/python3 without
    pydantic, the same constraint typego_bringup.launch.py works around. Returns
    '' when unavailable; the node then falls back to $EDGE_SERVICE_IP / localhost.
    """
    path = _locate_robot_yaml()
    if not path:
        return ''
    try:
        import yaml
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        edge = ((data.get('network') or {}).get('edge_service') or {})
        ip = str(edge.get('ip', '') or '').strip()
        if not ip:
            return ''
        port = str(edge.get('port', '') or '50049').strip()
        return f'http://{ip}:{port}/process'
    except Exception:
        return ''


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('waypoint_label'),
        'config', 'semantics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'vlm_endpoint',
            default_value=_edge_service_endpoint(),
            description='qwenvl /process endpoint; defaults to '
                        'network.edge_service in robot.yaml (override with '
                        'vlm_endpoint:=http://host:port/process).'),
        Node(
            package='waypoint_label',
            executable='semantic_labeler_node',
            name='semantic_labeler',
            output='screen',
            # The inline dict is applied after the YAML file, so a non-empty
            # vlm_endpoint (from robot.yaml or the CLI) overrides the file.
            parameters=[cfg, {
                'vlm_endpoint': LaunchConfiguration('vlm_endpoint'),
            }],
        ),
    ])
