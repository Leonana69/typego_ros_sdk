"""Service node smoke tests."""
import json

import pytest
import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from typego_config.loader import default_config_path
from typego_config.service_node import ConfigServiceNode


@pytest.fixture(scope='module')
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_session):
    n = ConfigServiceNode(default_config_path())
    yield n
    n.destroy_node()


def test_node_declares_expected_params(node):
    names = set(node._declared)
    assert 'network.ros_domain_id' in names
    assert 'profiles.nav2_params_file' in names
    assert 'web_gateway.bag_retain' in names


def test_every_parameter_is_read_only(node):
    """Static model: the resolved config is immutable for the launch's life.

    Previously `dynamic:` marked a handful of keys writable, but nothing
    downstream ever observed a change -- BagRotator snapshots its values at
    construction and slam_map_name is baked into the launch. The writable
    surface promised mutation it could not deliver, so it is gone.
    """
    for name in node._declared:
        assert node.describe_parameter(name).read_only is True, name


def test_integer_range_descriptor(node):
    desc = node.describe_parameter('network.ros_domain_id')
    assert desc.type == ParameterType.PARAMETER_INTEGER
    assert desc.integer_range
    ir = desc.integer_range[0]
    assert ir.from_value == 0 and ir.to_value == 232


def test_set_parameter_is_refused(node):
    """A read-only parameter cannot be mutated at runtime."""
    from rclpy.exceptions import ParameterImmutableException
    try:
        result = node.set_parameters([
            Parameter('web_gateway.bag_retain', Parameter.Type.INTEGER, 12),
        ])
    except ParameterImmutableException:
        pass
    else:
        assert result[0].successful is False
    assert node.get_parameter('web_gateway.bag_retain').value != 12


def test_reload_service_is_gone(node):
    """/typego_config/reload had zero callers and implied live mutation."""
    names = [n for n, _ in node.get_service_names_and_types()]
    assert not any(n.endswith('typego_config/reload') for n in names)
    assert any(n.endswith('typego_config/get_config') for n in names)


def test_get_config_service_returns_json(node):
    request = Trigger.Request()
    response = Trigger.Response()
    node._srv_get_config(request, response)
    assert response.success is True
    doc = json.loads(response.message)
    assert doc['robot']['type'] in ('go2', 'kami')
    assert 'network' in doc
