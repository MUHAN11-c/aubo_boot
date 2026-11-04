# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Opt-in standard service metadata introspection for isolated load diagnosis."""

import faulthandler
import os
import re
import signal

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.service_introspection import ServiceIntrospectionState

_installed = None


def install_service_trace(endpoint=None, *, trace_clients=True):
    """Enable the selected service only when a diagnostic run requests it."""
    global _installed
    if endpoint is None:
        endpoint = os.environ.get('WEB_LINK_TRACE_SERVICE', '')
    if not endpoint:
        return False
    if (not isinstance(endpoint, str) or re.fullmatch(
            r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', endpoint) is None):
        raise ValueError('WEB_LINK_TRACE_SERVICE must be an absolute ROS service name')
    if not isinstance(trace_clients, bool):
        raise TypeError('trace_clients must be boolean')
    key = (endpoint, trace_clients)
    if _installed is not None:
        if _installed != key:
            raise RuntimeError('Cannot retarget an installed diagnostic trace')
        return True
    original_client, original_service = Node.create_client, Node.create_service
    # Metadata contains phases, timestamps, GID and sequence, never application
    # request/response values. A retained event history helps the late observer;
    # short-lived client publishers can still vanish before DDS discovery.
    event_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def enable(owner, name, entity):
        if owner.resolve_service_name(name) == endpoint:
            entity.configure_introspection(owner.get_clock(), event_qos,
                                           ServiceIntrospectionState.METADATA)
        return entity

    def create_client(self, srv_type, srv_name, **kwargs):
        return enable(self, srv_name, original_client(self, srv_type, srv_name, **kwargs))

    def create_service(self, srv_type, srv_name, callback, **kwargs):
        entity = original_service(self, srv_type, srv_name, callback, **kwargs)
        return enable(self, srv_name, entity)

    # Diagnostic signal only, enabled in these owned processes before startup.
    # It writes Python stacks to the existing launch stderr, never stops ROS.
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    if trace_clients:
        # The managed pool validates both this exact wrapper and its original;
        # a marker alone never authorizes an unknown replacement function.
        create_client._web_link_trace_original = original_client
        Node.create_client = create_client
    Node.create_service = create_service
    _installed = key
    return True
