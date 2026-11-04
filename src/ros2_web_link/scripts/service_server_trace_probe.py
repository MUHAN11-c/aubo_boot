#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check a server-only metadata factory without starting a ROS context."""

import inspect
import os

from rclpy.node import Node
from service_trace import install_service_trace

assert 'trace_clients' in inspect.signature(install_service_trace).parameters, (
    'Server-only metadata must leave the native client factory unchanged')


class Entity:
    """Count only explicit introspection configuration calls."""

    def __init__(self):
        self.calls = []

    def configure_introspection(self, *args):
        """Retain scalar configuration sent to this controlled entity."""
        self.calls.append(args)


class Owner:
    """Supply the two public Node operations used by the installer."""

    def resolve_service_name(self, value):
        """Keep the absolute test endpoint unchanged."""
        return value

    def get_clock(self):
        """No native clock or node is created."""
        return 'clock'


previous_client, previous_service = Node.create_client, Node.create_service
saved = os.environ.pop('WEB_LINK_TRACE_SERVICE', None)
try:
    Node.create_client = lambda *a, **kw: Entity()
    Node.create_service = lambda *a, **kw: Entity()
    native_client = Node.create_client
    assert install_service_trace('/rosapi/nodes', trace_clients=False) is True
    assert Node.create_client is native_client
    assert not Node.create_client(Owner(), object, '/rosapi/nodes').calls
    target = Node.create_service(Owner(), object, '/rosapi/nodes', lambda a, b: b)
    assert len(target.calls) == 1
    assert not Node.create_service(Owner(), object, '/other', lambda a, b: b).calls
    hooked = Node.create_service
    assert install_service_trace('/rosapi/nodes', trace_clients=False)
    assert Node.create_service is hooked
    try:
        install_service_trace('/rosapi/nodes', trace_clients=True)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Installed server-only mode was silently expanded')
    print('PASS server-only metadata, untouched client factory, confinement and fixed mode')
finally:
    Node.create_client, Node.create_service = previous_client, previous_service
    if saved is not None:
        os.environ['WEB_LINK_TRACE_SERVICE'] = saved
