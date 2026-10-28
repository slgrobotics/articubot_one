"""Test articubot_one's robot-specific adapter to map_orientation_saver.

These tests exercise launch/configuration wiring without starting hardware or
copying estimator behavior tests from the standalone package. Run after sourcing
ROS: python3 -m pytest test/test_map_orientation_launch.py -q.
"""
import importlib.util
import math
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import yaml


ROOT = Path(__file__).parents[1]


def load_launch(relative):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location('adapter_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def adapter():
    return load_launch('launch/map_orientation.launch.py')


@pytest.fixture
def context(adapter):
    context = LaunchContext()
    for action in adapter.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


@pytest.fixture
def shares(tmp_path):
    locations = {name: tmp_path / name for name in ('articubot_one', 'map_orientation_saver')}
    navsat = locations['articubot_one'] / 'robots/dragger/config/navsat_transform.yaml'
    navsat.parent.mkdir(parents=True)
    navsat.write_text(yaml.safe_dump({'navsat_transform': {'ros__parameters': {
        'magnetic_declination_radians': .0730129404, 'yaw_offset': .1}}}))
    defaults = locations['map_orientation_saver'] / 'config/map_orientation.yaml'
    defaults.parent.mkdir(parents=True)
    defaults.write_text('/**/map_orientation_node:\n  ros__parameters: {}\n')
    return locations


def start(adapter, context, shares):
    with patch.object(adapter, 'get_package_share_directory', side_effect=lambda name: str(shares[name])), \
         patch.object(adapter, 'Node') as node, \
         patch.object(adapter, 'namespace_wrap') as namespace:
        adapter._start(context)
    return node.call_args.kwargs, namespace.call_args.args[0].perform(context)


def test_disabled_adapter_does_not_resolve_packages(adapter, context):
    context.launch_configurations['use_map_orientation_estimator'] = 'false'
    with patch.object(adapter, 'get_package_share_directory') as lookup:
        assert adapter._start(context) == []
        lookup.assert_not_called()


def test_dragger_starts_external_node_and_reuses_navsat(adapter, context, shares):
    context.launch_configurations.update(robot_model='dragger', namespace='robot1')
    kwargs, namespace = start(adapter, context, shares)
    assert kwargs['package'] == 'map_orientation_saver'
    assert kwargs['executable'] == 'map_orientation_node'
    assert kwargs['parameters'][0] == str(shares['map_orientation_saver'] / 'config/map_orientation.yaml')
    assert kwargs['parameters'][1] == dict(magnetic_declination_degrees=math.degrees(.0730129404), yaw_offset=.1)
    assert kwargs['parameters'][-1] == {'use_sim_time': False}
    assert kwargs['remappings'] == [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    assert namespace == 'robot1'


def test_seggy_hardware_uses_standalone_defaults(adapter, context, shares):
    context.launch_configurations['robot_model'] = 'seggy'
    kwargs, _ = start(adapter, context, shares)
    assert kwargs['parameters'] == [str(shares['map_orientation_saver'] / 'config/map_orientation.yaml'),
                                    {'use_sim_time': False}]


def test_simulation_selects_enu_before_explicit_override(adapter, context, shares, tmp_path):
    # An invalid hardware file must not be read in simulation.
    (shares['articubot_one'] / 'robots/dragger/config/navsat_transform.yaml').write_text('invalid: [')
    config = tmp_path / 'custom.yaml'
    config.write_text('/**/map_orientation_node:\n  ros__parameters:\n    yaw_offset: 0.2\n')
    context.launch_configurations.update(robot_model='dragger', use_sim_time='true',
                                         orientation_config_filepath=str(config))
    kwargs, _ = start(adapter, context, shares)
    assert kwargs['parameters'][1] == dict(heading_reference='true_enu',
                                          magnetic_declination_degrees=0., yaw_offset=0.)
    assert kwargs['parameters'][2] == str(config)
    assert kwargs['parameters'][-1] == {'use_sim_time': True}


@pytest.mark.parametrize('relative', [
    'launch/localizers.launch.py',
    'robots/dragger/launch/dragger.localizers.launch.py',
    'robots/seggy/launch/seggy.localizers.launch.py',
    'robots/dragger/launch/dragger.launch.py',
    'robots/seggy/launch/seggy.launch.py',
])
def test_robot_entry_points_preserve_enable_and_override_arguments(relative):
    description = load_launch(relative).generate_launch_description()
    args = {action.name: action for action in description.entities if isinstance(action, DeclareLaunchArgument)}
    context = LaunchContext()
    assert ''.join(value.perform(context) for value in args['use_map_orientation_estimator'].default_value) == 'true'
    assert ''.join(value.perform(context) for value in args['orientation_config_filepath'].default_value) == ''


def test_runtime_dependency_is_declared():
    package = ET.parse(ROOT / 'package.xml').getroot()
    assert 'map_orientation_saver' in [node.text for node in package.findall('exec_depend')]
