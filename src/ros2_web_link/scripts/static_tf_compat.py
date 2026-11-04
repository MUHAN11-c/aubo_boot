# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Keep rosbridge's late static-TF readers alive for every DDS writer's replay."""

from collections.abc import Callable
from dataclasses import dataclass
from functools import partial

from rclpy.qos import DurabilityPolicy
from rclpy.subscription import Subscription
from rosbridge_library.internal import subscribers
from rosbridge_library.internal.outgoing_message import OutgoingMessage
from tf2_msgs.msg import TFMessage


@dataclass
class _StaticTfClient:
    callback: Callable[[OutgoingMessage], None]
    subscription: Subscription | None = None


class _StaticTfMultiSubscriber(subscribers.MultiSubscriber):
    """Use upstream routing except for transient-local TFMessage subscribers."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._static_tf = (
            self.msg_class is TFMessage
            and self.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._static_tf_clients: dict[str, _StaticTfClient] = {}

    def subscribe(self, client_id, callback):
        """Retain one independent DDS reader for each late static-TF client."""
        if not self._static_tf:
            return super().subscribe(client_id, callback)
        with self.rlock:
            # A repeated subscribe updates the same route, rather than creating
            # another DDS reader or delivering each message twice.
            if client_id in self.subscriptions:
                self.subscriptions[client_id] = callback
                return
            client = self._static_tf_clients.get(client_id)
            if client is not None:
                client.callback = callback
                return

            # Upstream 2.7.1 merges a late client into the original reader after
            # its first sample. /tf_static has multiple retained DDS writers;
            # deleting that late reader loses all remaining writers' samples.
            # Preserve DDS replay and future updates without caching TF here.
            client = _StaticTfClient(callback)
            self._static_tf_clients[client_id] = client
            try:
                client.subscription = self.node_handle.create_subscription(
                    self.msg_class, self.topic,
                    partial(self._static_tf_callback, client_id=client_id, client=client),
                    qos_profile=self.qos_profile,
                    raw=self.raw,
                    callback_group=self.callback_group,
                )
            except Exception:
                del self._static_tf_clients[client_id]
                raise

    def _static_tf_callback(self, msg, *, client_id, client):
        with self.rlock:
            # Destruction runs later on the executor. Ignore in-flight callbacks
            # from a removed reader, including an unsubscribe/re-subscribe cycle.
            if self._static_tf_clients.get(client_id) is client:
                self.callback(msg, [client.callback])

    def unsubscribe(self, client_id):
        """Remove the route before scheduling its reader's destruction."""
        if not self._static_tf:
            return super().unsubscribe(client_id)
        with self.rlock:
            super().unsubscribe(client_id)
            client = self._static_tf_clients.pop(client_id, None)
            if client is not None and client.subscription is not None:
                self._schedule_destroy_subscription(client.subscription)
            # Late clients have their own readers, so the original DDS reader
            # can go away as soon as the original client leaves.
            if not self.subscriptions and self.subscriber is not None:
                self._schedule_destroy_subscription(self.subscriber)
                self.subscriber = None

    def has_subscribers(self):
        """Include clients that own independent static-TF readers."""
        if not self._static_tf:
            return super().has_subscribers()
        with self.rlock:
            return bool(self._static_tf_clients) or super().has_subscribers()

    def unregister(self):
        """Release all independent readers with the upstream executor helper."""
        if not self._static_tf:
            return super().unregister()
        with self.rlock:
            clients = list(self._static_tf_clients.values())
            self._static_tf_clients.clear()
            for client in clients:
                if client.subscription is not None:
                    self._schedule_destroy_subscription(client.subscription)
            if self.subscriber is not None:
                super().unregister()
                self.subscriber = None


def install_static_tf_compat():
    """Install the process-local compatibility class before the server starts."""
    subscribers.MultiSubscriber = _StaticTfMultiSubscriber
