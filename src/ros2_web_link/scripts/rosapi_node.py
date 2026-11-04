#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Run upstream rosapi with the public Jazzy action-graph compatibility API."""

import os
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_prefix
from rclpy.action.graph import get_action_names_and_types
from rclpy.node import Node
from rosapi import proxy
from service_trace import install_service_trace


def main():
    """Keep upstream services; avoid a mismatched ros2action Node method call."""
    server_endpoint = os.environ.get('WEB_LINK_TRACE_SERVER_SERVICE', '')
    if server_endpoint:
        if os.environ.get('WEB_LINK_TRACE_SERVICE'):
            raise ValueError('Choose full metadata or server-only metadata, not both')
        install_service_trace(server_endpoint, trace_clients=False)
    else:
        install_service_trace()
    # Installed rosapi 2.7.1 imports ros2action.api.get_action_names_and_types,
    # which calls Node.get_action_names_and_types on this host. Jazzy rclpy
    # instead exposes the public rclpy.action.graph function. Without this
    # adapter /rosapi/action_type raises AttributeError and kills rosapi.
    # Remove when the installed upstream pair supports the Node method.
    if not hasattr(Node, 'get_action_names_and_types'):
        proxy.get_action_names_and_types = get_action_names_and_types
    executable = Path(get_package_prefix('rosapi')) / 'lib' / 'rosapi' / 'rosapi_node'
    runpy.run_path(str(executable), run_name='__main__')


if __name__ == '__main__':
    main()
