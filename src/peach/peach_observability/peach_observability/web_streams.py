"""ROS topics for the existing browser projections; no command processing."""

import json

from std_msgs.msg import String

WEB_TOPICS = {
    'state': '~/web_state',
    'force': '~/web_force',
    'trajectory': '~/web_trajectory',
}


class WebStreams:
    """Publish the same read models used by HTTP for rosbridge subscribers."""

    def __init__(self, node):
        """Own publishers and timers for one active observability session."""
        self.node = node
        self.publishers = {
            key: node.create_publisher(String, name, 1)
            for key, name in WEB_TOPICS.items()}
        self.timers = [
            node.create_timer(1.0, self.state),
            node.create_timer(0.25, self.fast),
        ]

    def publish(self, key, projection):
        """Build a snapshot only for current subscribers."""
        publisher = self.publishers[key]
        if not publisher.get_subscription_count():
            return
        try:
            payload = json.dumps(projection(), ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError) as error:
            self.node.get_logger().warning(
                f'web_{key} snapshot rejected: {error}', throttle_duration_sec=5.0)
            return
        publisher.publish(String(data=payload))

    def state(self):
        """Publish state independently of the fast visualization projections."""
        self.publish('state', self.node.snapshot)

    def fast(self):
        """Keep force source age and trajectory cadence visible to the browser."""
        self.publish('force', self.node.force)
        self.publish('trajectory', self.node.trajectory)

    def close(self):
        """Release all resources on deactivate/shutdown."""
        for timer in self.timers:
            self.node.destroy_timer(timer)
        self.timers.clear()
        for publisher in self.publishers.values():
            self.node.destroy_publisher(publisher)
        self.publishers.clear()
