"""Optional named docking integration; merge fresh from persistent source YAMLs."""
import json
import os
import tempfile
import yaml
from launch.actions import SetLaunchConfiguration, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_docking(context):
    if LaunchConfiguration('enable_docking').perform(context).lower() != 'true':
        return []
    # Optional dependency: other robot bringups do not need detector packages.
    from dock_detector.nav2_config_merger import merge_nav2
    base_path = LaunchConfiguration('nav2_params_file').perform(context)
    catalogue_path = LaunchConfiguration('docking_catalogue').perform(context)
    settings_path = LaunchConfiguration('docking_params_file').perform(context)
    with open(base_path) as stream:
        base = yaml.safe_load(stream)
    with open(catalogue_path) as stream:
        catalogue = yaml.safe_load(stream)
    with open(settings_path) as stream:
        settings = yaml.safe_load(stream)
    merged = merge_nav2(base, catalogue, settings)
    # A runtime launch artifact, not a separately maintained full Nav2 snapshot.
    with tempfile.NamedTemporaryFile(mode='w', prefix='seggy-nav2-docking-', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(json.loads(json.dumps(merged)), stream, sort_keys=False)
        path = stream.name

    def cleanup(_context):
        if os.path.exists(path):
            os.unlink(path)
        return []

    parameters = dict(catalogue['dock_detector']['ros__parameters'])
    parameters['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    namespace = LaunchConfiguration('namespace', default='').perform(context).strip('/')
    return [
        SetLaunchConfiguration('nav2_params_file', path),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='dock_detector', executable='dock_detector', name='dock_detector',
             namespace='/'+namespace if namespace else '/', parameters=[parameters], output='screen',
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ]
