#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Run upstream rosbridge with narrow Jazzy message and static-TF fixes."""

from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import runpy
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from rosbridge_library.capabilities.fragmentation import Fragmentation
from rosbridge_library.internal import message_conversion

from static_tf_compat import install_static_tf_compat


def install_fragment_id_compat():
    """
    Give each 2.7.1 physical message its own outer fragment ID.

    Protocol.send creates a new Fragmentation instance per message. Its local
    seed therefore restarts at 0, mixing concurrent image/cloud streams. The
    fragment envelope ID is separate from the inner RPC/action interaction ID;
    serialize's cid argument is logging context only in this pinned version.
    Recheck/remove this adapter on upstream upgrade; do not patch system files.
    """
    try:
        if version('rosbridge-library') != '2.7.1':
            return False
    except PackageNotFoundError:
        return False
    if getattr(Fragmentation.fragment, '_web_link_unique_ids', False):
        return True
    original = Fragmentation.fragment

    def fragment(self, message, fragment_size, mid=None):
        return original(self, message, fragment_size, mid=uuid4().hex)

    fragment._web_link_unique_ids = True
    Fragmentation.fragment = fragment
    return True


def main():
    """Preserve the upstream server, protocol and ordinary topic routing."""
    from service_client_compat import install_service_clients_compat
    from service_trace import install_service_trace
    from service_wait_trace import install_service_wait_trace
    executable = Path(get_package_prefix('rosbridge_server')) / 'lib' / (
        'rosbridge_server') / 'rosbridge_websocket'
    upstream = runpy.run_path(str(executable), run_name='ros2_web_link_upstream')
    install_service_clients_compat(upstream)
    install_service_trace()
    install_service_wait_trace()
    install_static_tf_compat()
    install_fragment_id_compat()
    original = message_conversion._from_list_inst

    def from_list(inst, rostype):
        # Jazzy ParameterValue.byte_array_value is sequence<octet>, represented
        # by rclpy as list[bytes]. Upstream 2.7.1 shortcuts all primitive arrays
        # to list(inst), bypassing its scalar octet -> int conversion. JSON then
        # fails with "reject_bytes". Apply the existing scalar conversion only
        # to this list type; uint8/image/PointCloud2 and all others stay upstream.
        if message_conversion._remove_list_indicators(rostype) == 'octet':
            return [message_conversion._from_primitive_inst(item, 'octet')
                    for item in inst]
        return original(inst, rostype)

    message_conversion._from_list_inst = from_list
    upstream['main']()


if __name__ == '__main__':
    main()
