#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check trace opt-in and endpoint confinement without opening a ROS context."""

import os

from rclpy.node import Node
from rclpy.service_introspection import ServiceIntrospectionState
from service_trace import install_service_trace


class Entity:
    """Capture the public introspection configuration call."""

    def __init__(self):
        self.calls = []

    def configure_introspection(self, *args):
        """Record parameters without a DDS entity."""
        self.calls.append(args)


class Owner:
    """Provide the two public Node methods needed by the adapter."""

    def resolve_service_name(self, value):
        """Keep absolute test names unchanged."""
        return value

    def get_clock(self):
        """Return a distinct clock sentinel."""
        return 'clock'


previous_client, previous_service = Node.create_client, Node.create_service
previous_env = os.environ.pop('WEB_LINK_TRACE_SERVICE', None)
try:
    Node.create_client = lambda self, *args, **kwargs: Entity()
    Node.create_service = lambda self, *args, **kwargs: Entity()
    client_factory, service_factory = Node.create_client, Node.create_service
    assert install_service_trace() is False
    assert Node.create_client is client_factory
    os.environ['WEB_LINK_TRACE_SERVICE'] = 'relative'
    try:
        install_service_trace()
    except ValueError:
        pass
    else:
        raise AssertionError('Relative diagnostic endpoint must be rejected')
    assert Node.create_client is client_factory
    os.environ['WEB_LINK_TRACE_SERVICE'] = '/rosapi/nodes'
    assert install_service_trace() is True
    hooked = Node.create_client
    assert install_service_trace() is True and Node.create_client is hooked
    for entity in (Node.create_client(Owner(), object, '/rosapi/nodes'),
                   Node.create_service(Owner(), object, '/rosapi/nodes', lambda a, b: b)):
        assert len(entity.calls) == 1
        assert entity.calls[0][0] == 'clock'
        assert entity.calls[0][2] == ServiceIntrospectionState.METADATA
    assert not Node.create_client(Owner(), object, '/other').calls
    assert not Node.create_service(Owner(), object, '/other', lambda a, b: b).calls
    assert Node.create_client(Owner(), srv_type=object, srv_name='/rosapi/nodes').calls
    assert Node.create_service(Owner(), srv_type=object, srv_name='/rosapi/nodes',
                               callback=lambda a, b: b).calls
    assert not Node.create_client(Owner(), srv_type=object, srv_name='/other').calls
    assert not Node.create_service(Owner(), srv_type=object, srv_name='/other',
                                   callback=lambda a, b: b).calls
    assert Node.create_client(self=Owner(), srv_type=object, srv_name='/rosapi/nodes').calls
    print('PASS diagnostic opt-in, endpoint, idempotence, native metadata and confinement')
finally:
    Node.create_client, Node.create_service = previous_client, previous_service
    if previous_env is None:
        os.environ.pop('WEB_LINK_TRACE_SERVICE', None)
    else:
        os.environ['WEB_LINK_TRACE_SERVICE'] = previous_env
