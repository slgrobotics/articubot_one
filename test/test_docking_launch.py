"""
Verify that Seggy bringup merges docking settings into the Nav2 configuration.

The tests cover disabled docking, hardware and simulation catalogues,
namespace handling, and preservation of the robot navigation parameters.

From the workspace root, source install/setup.bash and run:

     python3 -m pytest -q src/articubot_one/test/test_docking_launch.py

Expect pytest to pass when each environment selects the expected catalogue
and limits. The tests inspect launch configuration without starting Nav2.
"""

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
        docking_params_file=str(ROOT/'robots/seggy/config'/('docking_nav2_real.yaml' if real else 'docking_nav2_sim.yaml')))
    if namespace is not None:context.launch_configurations['namespace']=namespace
    original=yaml.safe_load(Path(context.launch_configurations['nav2_params_file']).read_text())
    with patch('articubot_one.launch_utils.docking.Node') as node:
        actions=prepare_docking(context)
    actions[0].execute(context)
    actions[-2].execute(context)
    assert context.launch_configurations['nav2_dock_action'] == 'dock_robot_position'
    generated=Path(context.launch_configurations['nav2_params_file'])
    try:
        config=yaml.safe_load(generated.read_text())
        for key in original:
            if key!='docking_server':assert config[key]==original[key]
        server=config['docking_server']['ros__parameters']
        assert server['controller_frequency']==20.
        assert server['max_retries']==(0 if real else 2)
        ids=['real_v_bay'] if real else ['sim_trapezoid_bay','sim_v_bay']
        assert server['docks']==ids
        for dock_id in ids:
            assert server[dock_id]['id']==dock_id
            assert server[dock_id]['type']=='lidar_dock'
        if real:
            assert server['controller']['v_linear_max']==.0406
            assert server['real_v_bay']['frame']=='map'
            catalogue = yaml.safe_load(Path(context.launch_configurations['docking_catalogue']).read_text())
            prior = catalogue['dock_detector']['ros__parameters']['docks']['real_v_bay']
            assert server['real_v_bay']['pose'][0] == pytest.approx(prior['expected_dock_x'])
        assert 'dock_backwards' not in server
        assert server['controller']['costmap_topic']=='local_costmap/costmap_raw'
        assert node.call_args.kwargs['namespace']==('/robot_a' if namespace else '/')
        assert server['lidar_dock']['staging_x_offset']==-.5
        assert server['lidar_dock']['dock_direction']=='forward'
        assert server['lidar_dock']['staging_yaw_offset']==0.0
        assert server['lidar_dock']['rotate_to_dock'] is False
        assert server['lidar_dock']['external_detection_timeout']==.5
        assert node.call_args.kwargs['parameters'][0]['use_sim_time'] is (not real)
        detector, wrapper = [call.kwargs for call in node.call_args_list]
        assert detector['parameters'][0]['dock_ids'] == ids
        assert wrapper['executable'] == 'docking_alignment'
        assert wrapper['parameters'][0]['docking_threshold_yaw'] == 5.0
        assert wrapper['parameters'][0]['docking_threshold'] == server['lidar_dock']['docking_threshold']
        assert wrapper['parameters'][0]['trace_enabled'] is True
        assert wrapper['parameters'][0]['alignment_position_margin'] == .02
        assert wrapper['parameters'][0]['dock_ids'] == ids
        for dock_id in ids:
            catalogue = yaml.safe_load(Path(context.launch_configurations['docking_catalogue']).read_text())
            expected_delay = catalogue['dock_detector']['ros__parameters']['docks'][dock_id].get('charging_to_stop_delay', 0.0)
            assert wrapper['parameters'][0]['docks'][dock_id]['charging_to_stop_delay'] == expected_delay
        alignment = wrapper['parameters'][0]
        assert alignment['insertion_min_speed'] == alignment['insertion_speed'] == .0406
        assert alignment['minimum_angular_speed_deg'] == 9.84
        assert server['controller']['v_linear_min'] == .0406
        if real:
            assert alignment.get('insertion_max_travel', 0.0) == 0.0
            catalogue = yaml.safe_load(Path(context.launch_configurations['docking_catalogue']).read_text())
            expected = catalogue['dock_detector']['ros__parameters']['docks']['real_v_bay']
            assert alignment['docks']['real_v_bay']['insertion_max_travel'] == expected['insertion_max_travel']
        else:
            assert alignment['insertion_max_travel'] == 0.05
        assert server['lidar_dock']['docking_threshold'] == .025
        assert server['lidar_dock']['trace_final_yaw_deg'] == 5.0
        assert server['lidar_dock']['docking_threshold_yaw'] == 180.0
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
        'docking_nav2_sim.yaml' if sim else 'docking_nav2_real.yaml')
