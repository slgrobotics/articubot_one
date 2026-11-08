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
@pytest.mark.parametrize("real", [False, True])
def test_enabled_merge_uses_current_robot_params_and_catalogue(namespace, real):
    context=LaunchContext()
    context.launch_configurations.update(enable_docking='true',use_sim_time='false' if real else 'true',
        nav2_params_file=str(ROOT/'robots/seggy/config/nav2_params.yaml'),
        docking_catalogue=str(ROOT/'robots/seggy/config'/('docking_real.yaml' if real else 'docking_sim.yaml')),
        docking_params_file=str(ROOT/'robots/seggy/config'/('docking_nav2_real.yaml' if real else 'docking_nav2.yaml')))
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
        assert server['max_retries']==(0 if real else 2)
        ids=['real_v_bay'] if real else ['trapezoid_bay','v_bay']
        assert server['docks']==ids
        for dock_id in ids:
            assert server[dock_id]['id']==dock_id
            assert server[dock_id]['type']=='lidar_dock'
        if real:
            assert server['controller']['v_linear_max']==.015
            assert server['real_v_bay']['frame']=='map'
            assert server['real_v_bay']['pose'][0]==pytest.approx(2.361164737773733)
        assert 'dock_backwards' not in server
        assert server['controller']['costmap_topic']=='local_costmap/costmap_raw'
        assert node.call_args.kwargs['namespace']==('/robot_a' if namespace else '/')
        assert server['lidar_dock']['staging_x_offset']==-.5
        assert server['lidar_dock']['dock_direction']=='forward'
        assert server['lidar_dock']['staging_yaw_offset']==0.0
        assert server['lidar_dock']['rotate_to_dock'] is False
        assert server['lidar_dock']['external_detection_timeout']==.5
        assert node.call_args.kwargs['parameters'][0]['use_sim_time'] is (not real)
        assert node.call_args.kwargs['parameters'][0]['dock_ids']==ids
    finally:generated.unlink()


@pytest.mark.parametrize('sim_time', ['true', 'false'])
def test_seggy_launch_selects_environment_catalogue_and_limits(sim_time):
    import importlib.util
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    spec=importlib.util.spec_from_file_location(
        'seggy_docking_launch_test', ROOT/'robots/seggy/launch/seggy.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    declarations={a.name:a for a in module.generate_launch_description().entities
                  if isinstance(a,DeclareLaunchArgument)}
    context=LaunchContext();context.launch_configurations['use_sim_time']=sim_time
    sim=sim_time=='true'
    assert perform_substitutions(context,declarations['enable_docking'].default_value)==sim_time
    assert Path(perform_substitutions(context,declarations['docking_catalogue'].default_value)).name==(
        'docking_sim.yaml' if sim else 'docking_real.yaml')
    assert Path(perform_substitutions(context,declarations['docking_params_file'].default_value)).name==(
        'docking_nav2.yaml' if sim else 'docking_nav2_real.yaml')
