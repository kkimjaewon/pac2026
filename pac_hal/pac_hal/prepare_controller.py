"""Configure and atomically switch only the verified Gazebo robot controllers."""
import argparse
from pathlib import Path
import tempfile

from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController
from controller_manager_msgs.srv import SwitchController, UnloadController
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.node import Node
import yaml

from .contract import load_config
from .kinematics import Kinematics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-file', required=True)
    parser.add_argument('--description-output', type=Path)
    args, ros_args = parser.parse_known_args()
    config = load_config(args.config_file)
    session = config['runtime']['active_session_id']
    if session <= 0:
        raise ValueError('Gazebo execution requires a nonzero session_id')
    rclpy.init(args=ros_args)
    node = Node('prepare_hal_controller')

    def call(service, name, request):
        client = node.create_client(service, name)
        try:
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('service unavailable: ' + name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30)
            if not future.done() or future.exception() is not None:
                raise RuntimeError('service call failed: ' + name)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        req = GetParameters.Request()
        req.names = [config['robot']['robot_description_parameter']]
        response = call(GetParameters, '/robot_state_publisher/get_parameters', req)
        xml = response.values[0].string_value
        model = Kinematics(xml, config)  # Checks Gazebo backend/model chain before any switch.
        # Bind the gateway to the same description used to prepare receiver
        # limits. Failure to save it must occur before any controller switch.
        if args.description_output is not None:
            args.description_output.write_text(xml, encoding='utf-8')
        manager = config['transport']['controller_manager']
        name = config['transport']['controller_name']
        current = call(ListControllers, manager+'/list_controllers', ListControllers.Request())
        existing = next((c for c in current.controller if c.name == name), None)
        if existing is not None:
            if existing.state == 'active':
                req = SwitchController.Request()
                req.deactivate_controllers = [name]
                req.strictness = req.STRICT
                req.timeout.sec = 10
                if not call(SwitchController, manager+'/switch_controller', req).ok:
                    raise RuntimeError('could not deactivate existing HAL controller')
            req = UnloadController.Request()
            req.name = name
            if not call(UnloadController, manager+'/unload_controller', req).ok:
                raise RuntimeError('could not unload previous HAL controller')
        axes = sorted(config['joints'], key=lambda j: j['device_index'])
        canonical_index = {n: i for i, n in enumerate(model.names)}
        lower, upper = [], []
        for axis in axes:
            index = canonical_index[axis['name']]
            ends = [axis['direction']*v + axis['zero_offset_rad']
                    for v in (model.lower[index], model.upper[index])]
            lower.append(min(ends))
            upper.append(max(ends))
        controller_parameters = {'joints': [j['name'] for j in axes],
                                 'direction': [j['direction'] for j in axes],
                                 'zero_offset': [j['zero_offset_rad'] for j in axes],
                                 'lower': lower, 'upper': upper,
                                 'max_velocity': [j['max_velocity_rad_s'] for j in axes],
                                 'max_acceleration': [j['max_acceleration_rad_s2'] for j in axes],
                                 'session_id': session,
                                 'command_ttl_ns': config['timing']['command_ttl_ns'],
                                 'watchdog_ns': config['timing']['receiver_watchdog_ns']}
        folder = Path(tempfile.mkdtemp(prefix='pac_hal_controller_'))
        params = folder/'controller.yaml'
        params.write_text(yaml.safe_dump({name: {'ros__parameters': controller_parameters}}))
        req = SetParametersAtomically.Request()
        req.parameters = [
            RosParameter(name=name+'.type', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='pac_hal/WatchdogPositionController')),
            RosParameter(name=name+'.params_file', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=[str(params)])),
        ]
        outcome = call(SetParametersAtomically, manager+'/set_parameters_atomically', req)
        if not outcome.result.successful:
            raise RuntimeError(outcome.result.reason)
        req = LoadController.Request()
        req.name = name
        if not call(LoadController, manager+'/load_controller', req).ok:
            raise RuntimeError('plugin load failed; restart Gazebo with rebuilt workspace sourced')
        req = ConfigureController.Request()
        req.name = name
        if not call(ConfigureController, manager+'/configure_controller', req).ok:
            raise RuntimeError('HAL controller configuration failed')
        current = call(ListControllers, manager+'/list_controllers', ListControllers.Request())
        wanted = set(model.names)
        deactivate = []
        for controller in current.controller:
            if controller.state != 'active' or controller.name == name:
                continue
            # Broadcasters claim no command interfaces and stay running.
            if any(interface.rsplit('/', 1)[0] in wanted
                   for interface in controller.claimed_interfaces):
                deactivate.append(controller.name)
        req = SwitchController.Request()
        req.activate_controllers = [name]
        req.deactivate_controllers = deactivate
        req.strictness, req.activate_asap = req.STRICT, True
        req.timeout.sec = 10
        if not call(SwitchController, manager+'/switch_controller', req).ok:
            raise RuntimeError('atomic controller switch failed')
        node.get_logger().info(f'Gazebo {name} active; session={session}; replaced={deactivate}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
