import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _edge_service_endpoint():
    """Build the qwenvl `/process` URL from robot.yaml's network.edge_service.

    Discovery comes from typego_config.bootstrap, which is dependency-free, so
    this does not require pydantic. Returns '' when unavailable; the node then
    falls back to $EDGE_SERVICE_IP / localhost.
    """
    try:
        from typego_config.bootstrap import load_merged
        data, _ = load_merged()
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
