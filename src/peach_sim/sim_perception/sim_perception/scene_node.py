"""Run the existing Peach scene perception node with simulation parameters."""


def main(args=None):
    """Forward ROS arguments to the shared capability entry point."""
    from peach_scene_perception.scene_perception_node import main as scene_main

    return scene_main(args=args)
