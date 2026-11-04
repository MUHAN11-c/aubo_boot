# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""
Bind the private pool to one reviewed rosbridge Node, preserving its main.

Node construction precedes executor attachment in upstream 2.7.1. Validate
deployment parameters before the upstream listener, then create a pool once on
the first worker request after add_node. Ordinary Nodes keep the upstream
function. Managed calls share admission even with cache0, whose per-call
clients use only public rclpy APIs. No global rclpy entity hooks are installed.
"""

import hashlib
from importlib.metadata import version
import inspect
import math
import os
from pathlib import Path
from threading import Lock

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.executors import Executor, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.task import Future, Task
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.internal import services
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

from service_budget import ServiceCallBudget
from service_clients import ServiceClientPool

_installed = None
_TRACE_CLIENT_HASH = '71a329837d8529f9af2301397343247180ceddef6924c4b24667b14ddae77846'
# Revalidated from the actual apt source, not from the differing upstream tag.
_RUNTIME_HASHES = {
    'Node.create_client':
        'e59c6c3065c5c72140fd180e9e39d3c9b39240d37677ab5bbbcd12c1c5107d7d',
    'Node.destroy_client':
        '8e8772398e8cf1e04384f48f2aee7960e4877e80c0e3faa4b95f288d170d3221',
    'Node._wake_executor':
        'de15c978481466bdf9d56dff107e230e944911a9a11e3f308f04ff147c07f03b',
    'Executor.create_task':
        'f534dc1c3ab6d636a8dd3ca8a8d23b0d3a72471586ed5b047367d361d5f35f7f',
    'Executor.wake':
        '4b260415fcd8298d4bcc8cd64360d101f9afdb7872e43b719d6697b17817e2b8',
    'Executor._wait_for_ready_callbacks':
        'ed7ac8f497223079edceff91c93237e050201735cfb635451dcfb167d514226d',
    'Executor._make_handler':
        '2594c098403ee628c07d418a42f1a172b7cf0d83c03524963b243f249875735b',
    'Executor._take_client':
        'a82a66e17a9efe0adcff747f241711c4fe44c7f41def3514aef781e23273a662',
    'SingleThreadedExecutor._spin_once_impl':
        '420a30caf0b94f4386ebb2977a11c47727c7b4df1e9bd83d558085590439e7d3',
    'MultiThreadedExecutor._spin_once_impl':
        '518099bed324ecdc281af2e9fa7fa3398eb9823aab03a024dbb5d23b1f37cda3',
    'Client.call_async':
        'e6cd96b1da9eabec6fd3de307db8ecae5cf4262b5fed4e5c0e535b17637219be',
    'Client.remove_pending_request':
        'e25a1c61fc7e438f64bff6980365638a878caf3b5947f727d4995a517d298692',
    'Client.destroy':
        '56f375a33c36889eeb84bf98d7f747fccd728a4e4ffd67a2ddc304eb86b7040d',
    'Task.__call__':
        'effee93f36517e9162018798f7c49caed1a13931c828aaf3fd9c2ca4bfd99864',
    'Future.add_done_callback':
        '44f754cad811c3008c2b49ee46d3b5bbdc7d8a483e0ffbd88511caf7dab2f668',
    'Future.cancel':
        '946e3a19e556ca30293693d480cf89fe433f0466d896c06d747cc66bdc13ef47',
    'MutuallyExclusiveCallbackGroup.can_execute':
        'cbda0783bb73af8ae0b4d50ddef715b5903a81c76a958703173f587c352aee1c',
    'MutuallyExclusiveCallbackGroup.beginning_execution':
        '00d9dfbfef54d190dca53fab90aecec38529f098fc253a6125acc3b17d3ef431',
    'MutuallyExclusiveCallbackGroup.ending_execution':
        '51fc72b8773e58fb09337285eee824834ed66737a1b413687cd4a9dd194618b0',
}
_UPSTREAM_HASHES = {
    'main':
        'c955915ce2c528e9917445de7c16acfc099515fd85aa7ec60095b4a33ccfd206',
    'async_main':
        '9b5e827324aae98e7e6113b09d9f5bb93a3f3c40bb5869c29bc53cabaa1271f8',
    'RosbridgeWebsocketNode.__init__':
        '38f4a128de244adbf1f74bca4d4fae6446979cddf3fc99eda91e1a76699eb301',
    'RosbridgeWebsocketNode._handle_parameters':
        'ffa497a4f5fecf8c7e308cae1ddfb1750969487b94e7cbfb4ddb844a537bce80',
    'RosbridgeWebsocketNode._start_server':
        'c1f4471dcf3f19b222484c47b3cb634dca11d35b455e784a3ee0ed77e8b90167',
    'services.call_service':
        '6671ca5f152f848a14bd2ed52521b1ea868ff9204dd3fa169ce0b2492425f697',
}
_NATIVE_HASH = '53efd7622c109ae6f9ff7ffafc3bffac4856700d39ca51d8689ecaa617a596bd'
_CAPABILITY_HASHES = {
    '__init__': 'de0b2097f803124e91a981cb52d761d53d303f3e40acc5988073d26eedebdb12',
    'call_service': 'fa1e1f4921e5b1d8aa7ba986e1e7a03ee76b0644531c2a72fafb0ea402c80a6e',
}


def _source_matches(methods, expected):
    """Treat an uninspectable or modified method as an unsupported layout."""
    try:
        # getsource(function) unwraps decorators; the actual code must match.
        return all(
            hashlib.sha256(inspect.getsource(methods[name].__code__).encode()).hexdigest() == h
            for name, h in expected.items())
    except (OSError, TypeError, KeyError, AttributeError):
        return False


def _review_runtime():
    """Fail before listening when the owner dispatch/borrow layout is unknown."""
    classes = {cls.__name__: cls for cls in (
        Node, Executor, SingleThreadedExecutor, MultiThreadedExecutor, Client,
        Task, Future, MutuallyExclusiveCallbackGroup)}
    methods = {name: getattr(classes[name.split('.')[0]], name.split('.')[1])
               for name in _RUNTIME_HASHES}
    client_factory = methods['Node.create_client']
    traced_original = getattr(client_factory, '_web_link_trace_original', None)
    if traced_original is not None:
        if not _source_matches({'client': client_factory}, {'client': _TRACE_CLIENT_HASH}):
            raise RuntimeError('Service cache does not recognize this metadata client wrapper')
        methods['Node.create_client'] = traced_original
    if (version('rclpy') != '7.1.12' or
            hashlib.sha256(Path(rclpy_implementation.__file__).read_bytes()).hexdigest() !=
            _NATIVE_HASH or not _source_matches(methods, _RUNTIME_HASHES)):
        raise RuntimeError('Service cache requires the reviewed local rclpy 7.1.12 layout; '
                           'set service_client_cache_size=0 for the upstream path')


def install_service_clients_compat(upstream):
    """Install one thin managed class in both runpy's copy and actual globals."""
    global _installed
    namespace = upstream['async_main'].__globals__
    if _installed is not None:
        previous_namespace, managed = _installed
        if namespace is not previous_namespace:
            raise RuntimeError('Cannot retarget an installed managed bridge adapter')
        upstream['RosbridgeWebsocketNode'] = managed
        return managed
    original_class = upstream['RosbridgeWebsocketNode']
    original_service = services.call_service
    methods = {name: (getattr(original_class, name.split('.')[1])
                      if name.startswith('RosbridgeWebsocketNode.')
                      else original_service if name == 'services.call_service'
                      else upstream[name]) for name in _UPSTREAM_HASHES}
    if (version('rosbridge-library') != '2.7.1' or version('rosbridge-server') != '2.7.1' or
            upstream['main'].__globals__ is not namespace or
            namespace['SingleThreadedExecutor'] is not SingleThreadedExecutor or
            not _source_matches(methods, _UPSTREAM_HASHES) or
            not _source_matches({name: getattr(CallService, name)
                                 for name in _CAPABILITY_HASHES}, _CAPABILITY_HASHES) or
            CallService not in RosbridgeProtocol.rosbridge_capabilities):
        raise RuntimeError('Managed bridge requires the reviewed rosbridge 2.7.1 layout')

    class ManagedRosbridgeNode(original_class):
        """Keep Node-owned resources and deployment policy within this server."""

        def __init__(self):
            self._web_link_pool_guard = Lock()
            self._web_link_pool = None
            self._web_link_budget = None
            self._web_link_pool_closing = False
            super().__init__()

        def _handle_parameters(self):
            super()._handle_parameters()
            definitions = (
                ('service_client_cache_size', 0, 0, 1024,
                 'Node-owned service cache slots; 0 selects upstream per-call clients.'),
                ('service_client_max_in_flight', 128, 1, 4096,
                 'Maximum admitted bridge Service calls and pending entity cleanup.'),
                ('service_client_creation_timeout', 5.0, 0.0, 60.0,
                 'Creation/retirement scheduling deadline in monotonic seconds; finite >0.'),
            )
            values = []
            for name, default, minimum, maximum, description in definitions:
                descriptor = ParameterDescriptor(read_only=True, description=description)
                if type(default) is int:
                    descriptor.integer_range = [IntegerRange(
                        from_value=minimum, to_value=maximum, step=1)]
                else:
                    descriptor.floating_point_range = [FloatingPointRange(
                        from_value=minimum, to_value=maximum, step=0.0)]
                value = self.declare_parameter(name, default, descriptor).value
                valid = (type(value) is type(default) and minimum <= value <= maximum)
                if type(default) is float:
                    valid = valid and math.isfinite(value) and value > 0
                if not valid:
                    raise ValueError(f'Invalid deployment parameter {name}: {value}')
                values.append(value)
            self._web_link_pool_config = tuple(values)
            self._web_link_budget = ServiceCallBudget(self, values[1], values[2])
            if os.environ.get('WEB_LINK_LATE_CLIENT_SERVICE'):
                raise ValueError('Post-timeout reader retention is an archived upstream '
                                 'diagnostic; bounded Service calls do not retain late readers')
            if values[0]:
                if self.use_events_executor:
                    raise ValueError('Service cache does not support EventsExecutor; '
                                     'set service_client_cache_size=0')
                if not self.protocol_parameters['call_services_in_new_thread']:
                    raise ValueError('Service cache requires call_services_in_new_thread=true')
                _review_runtime()

        def _web_link_get_pool(self):
            # Construction has no ROS/native operation and is safe under this
            # private lock. Never hold it while running a service or pool.close.
            with self._web_link_pool_guard:
                if self._web_link_pool_closing:
                    raise RuntimeError('ROS node closing')
                if not self._web_link_pool_config[0]:
                    return None
                if self._web_link_pool is None:
                    if type(self.executor) not in (SingleThreadedExecutor, MultiThreadedExecutor):
                        raise RuntimeError('Service cache requires a reviewed attached executor')
                    slots, maximum, deadline = self._web_link_pool_config
                    self._web_link_pool = ServiceClientPool(
                        self, min(slots, maximum), maximum, deadline)
                return self._web_link_pool

        def destroy_node(self):
            """Wake local calls before standard Node cleanup, preserving failures."""
            with self._web_link_pool_guard:
                self._web_link_pool_closing = True
                pool = self._web_link_pool
            try:
                if self._web_link_budget is not None:
                    self._web_link_budget.close()
                if pool is not None:
                    pool.close()
            finally:
                super().destroy_node()

    def call_service(node_handle, service, args=None, server_ready_timeout=1.0,
                     server_response_timeout=5.0):
        """Keep cache policy separate from the shared admission/deadline policy."""
        if isinstance(node_handle, ManagedRosbridgeNode):
            def invoke(call):
                pool = node_handle._web_link_get_pool()
                if pool is not None:
                    return pool.call_service(
                        service, args, server_ready_timeout, server_response_timeout)
                return node_handle._web_link_budget.call_uncached(
                    call, service, args, server_ready_timeout, server_response_timeout)
            return node_handle._web_link_budget.run(invoke)
        return original_service(node_handle, service, args, server_ready_timeout,
                                server_response_timeout)

    class ManagedCallService(CallService):
        """Retain upstream permission/conversion/response handling after admission."""

        def __init__(self, protocol):
            super().__init__(protocol)
            if isinstance(protocol.node_handle, ManagedRosbridgeNode):
                protocol.register_operation('call_service', self._submit)

        def _submit(self, message):
            self.basic_type_check(message, self.call_service_msg_fields)
            cid, service = message.get('id'), message['service']

            def failure(error):
                self._failure(cid, service, error)

            try:
                self.protocol.node_handle._web_link_budget.submit(
                    lambda: self.call_service(message), failure)
            except Exception as error:
                failure(error)

    call_service._web_link_original = original_service
    call_service._web_link_response = ServiceClientPool.call_service
    call_service._web_link_uncached_response = ServiceCallBudget.call_uncached
    namespace['RosbridgeWebsocketNode'] = ManagedRosbridgeNode
    upstream['RosbridgeWebsocketNode'] = ManagedRosbridgeNode
    services.call_service = call_service
    RosbridgeProtocol.rosbridge_capabilities = tuple(
        ManagedCallService if capability is CallService else capability
        for capability in RosbridgeProtocol.rosbridge_capabilities)
    _installed = (namespace, ManagedRosbridgeNode)
    return ManagedRosbridgeNode
