"""Robot Wi-Fi V2 bringup waits normally for the first complete saved map."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
import yaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('robot_wifi_map_wait', ROOT / 'launch/wifi_survey_v2.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def context(tmp_path):
    config = tmp_path / 'wifi.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'map_yaml_path': str(tmp_path / 'later.yaml'), 'db_path': str(tmp_path / 'wifi.db'),
        'use_gps': True, 'map_check_interval': .1,
    }}}))
    context = LaunchContext()
    with patch.object(module, 'get_package_share_directory', return_value=str(tmp_path)):
        description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations['config_filepath'] = str(config)
    return context


def test_missing_map_defers_whole_suite_then_releases_it_once(context, tmp_path):
    actions = module._launch_nodes(context)
    assert len(actions) == 1 and isinstance(actions[0], OpaqueFunction)
    waiting = actions[0].execute(context)
    timer = next(action for action in waiting if isinstance(action, TimerAction))
    assert not any(isinstance(action, ExecuteProcess) for action in waiting)
    (tmp_path / 'later.yaml').write_text('image: later.pgm\nresolution: 0.25\norigin: [0,0,0]\n')
    (tmp_path / 'later.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    ready = list(timer.actions)[0].execute(context)
    assert any(isinstance(action, ExecuteProcess) for action in ready)
    assert not any(isinstance(action, TimerAction) for action in ready)
    assert actions[0].execute(context) == []


def test_disabled_suite_does_not_wait(context):
    context.launch_configurations.update(monitoring='false', survey='false', visualization='false')
    assert module._launch_nodes(context) == []


def test_polling_interval_override_wins_over_yaml(context):
    context.launch_configurations['map_check_interval'] = '2.5'
    with patch.object(module, 'wait_for_saved_map', return_value='gate') as gate:
        assert module._launch_nodes(context) == ['gate']
    assert gate.call_args.args[2] == '2.5'


def test_filename_still_required_to_know_which_map_to_watch(context):
    path = Path(context.launch_configurations['config_filepath'])
    path.write_text('/**:\n  ros__parameters: {}\n')
    with pytest.raises(RuntimeError, match='map_yaml_path'):
        module._launch_nodes(context)
