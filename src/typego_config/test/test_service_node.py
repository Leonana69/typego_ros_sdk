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


def test_read_only_flag_applied(node):
    desc = node.describe_parameter('network.ros_domain_id')
    assert desc.read_only is True
    desc = node.describe_parameter('web_gateway.bag_retain')
    assert desc.read_only is False


def test_integer_range_descriptor(node):
    desc = node.describe_parameter('network.ros_domain_id')
    assert desc.type == ParameterType.PARAMETER_INTEGER
    assert desc.integer_range
    ir = desc.integer_range[0]
    assert ir.from_value == 0 and ir.to_value == 232


def test_on_set_rejects_out_of_range(node):
    result = node.set_parameters([
        Parameter('web_gateway.bag_retain', Parameter.Type.INTEGER, -1),
    ])
    assert result and result[0].successful is False
    reason = result[0].reason.lower()
    assert 'range' in reason or 'validation' in reason


def test_on_set_accepts_valid(node):
    result = node.set_parameters([
        Parameter('web_gateway.bag_retain', Parameter.Type.INTEGER, 12),
    ])
    assert result[0].successful is True
    assert node.get_parameter('web_gateway.bag_retain').value == 12


def test_get_config_service_returns_json(node):
    request = Trigger.Request()
    response = Trigger.Response()
    node._srv_get_config(request, response)
    assert response.success is True
    doc = json.loads(response.message)
    assert doc['robot']['type'] in ('go2', 'kami')
    assert 'network' in doc
