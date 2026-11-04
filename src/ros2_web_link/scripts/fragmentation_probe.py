#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Check the real upstream fragmenter without a ROS node, socket or background task."""

import json

from bridge_node import install_fragment_id_compat
from rosbridge_library.capabilities.fragmentation import Fragmentation


class SerializationContext:
    """Supply only the upstream serializer/log boundary, not a replacement codec."""

    delay_between_messages = 0

    def serialize(self, message, cid=None):
        """Use JSON exactly as the pinned upstream protocol does."""
        return json.dumps(message)

    def log(self, level, message):
        """Keep this explicit CLI check quiet."""


def main():
    """Use fresh upstream fragmenters as Protocol.send does for every message."""
    assert install_fragment_id_compat(), 'Probe targets installed rosbridge-library 2.7.1'
    install_fragment_id_compat()  # Repeated installation must be harmless.
    context = SerializationContext()
    first = {'op': 'publish', 'topic': '/camera', 'msg': {'data': 'x' * 200}}
    second = {'op': 'service_response', 'id': 'request-id', 'result': True,
              'values': {'data': 'y' * 200}}
    one = list(Fragmentation(context).fragment(first, 32))
    two = list(Fragmentation(context).fragment(second, 32, mid='request-id'))
    assert len({p['id'] for p in one + two}) == 2
    for parts, original in ((one, first), (two, second)):
        assert json.loads(''.join(p['data'] for p in parts)) == original
        assert [p['num'] for p in parts] == list(range(len(parts)))
        assert all(p['total'] == len(parts) for p in parts)
    print('PASS unique physical fragment IDs; exact payload and RPC ID preserved; '
          'idempotent install; no process started')


if __name__ == '__main__':
    main()
