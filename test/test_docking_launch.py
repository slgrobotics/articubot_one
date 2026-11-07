"""Integration merge preserves working navigation and selects named simulation docks."""
from pathlib import Path
from unittest.mock import patch
import yaml
import pytest
from launch import LaunchContext
from articubot_one.launch_utils.docking import prepare_docking

ROOT=Path(__file__).parents[1]


def test_disabled_docking_does_not_read_files_or_start_nodes():
    context=LaunchContext();context.launch_configurations['enable_docking']='false'
    assert prepare_docking(context)==[]


@pytest.mark.parametrize("namespace", [None, "robot_a"])
def test_enabled_merge_uses_current_robot_params_and_catalogue(namespace):
    context=LaunchContext()
    context.launch_configurations.update(enable_docking='true',use_sim_time='true',
        nav2_params_file=str(ROOT/'robots/seggy/config/nav2_params.yaml'),
        docking_catalogue=str(ROOT/'robots/seggy/config/docking_sim.yaml'),
        docking_params_file=str(ROOT/'robots/seggy/config/docking_nav2.yaml'))
    if namespace is not None:context.launch_configurations['namespace']=namespace
    original=yaml.safe_load(Path(context.launch_configurations['nav2_params_file']).read_text())
    with patch('articubot_one.launch_utils.docking.Node') as node:
        actions=prepare_docking(context)
    actions[0].execute(context)
    generated=Path(context.launch_configurations['nav2_params_file'])
    try:
        config=yaml.safe_load(generated.read_text())
        for key in original:
            if key!='docking_server':assert config[key]==original[key]
        server=config['docking_server']['ros__parameters']
        assert server['controller_frequency']==20.
        assert server['max_retries']==2
        assert server['trapezoid_bay']['id']=='trapezoid_bay'
        assert server['v_bay']['id']=='v_bay'
        assert 'dock_backwards' not in server
        assert server['controller']['costmap_topic']=='local_costmap/costmap_raw'
        assert node.call_args.kwargs['namespace']==('/robot_a' if namespace else '/')
        assert server['lidar_dock']['staging_x_offset']==-.5
        assert server['lidar_dock']['external_detection_timeout']==.5
        assert node.call_args.kwargs['parameters'][0]['use_sim_time'] is True
        assert node.call_args.kwargs['parameters'][0]['dock_ids']==['trapezoid_bay','v_bay']
    finally:generated.unlink()
