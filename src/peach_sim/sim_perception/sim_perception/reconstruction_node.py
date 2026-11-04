"""Run the existing Peach target reconstruction node with simulation parameters."""


def main(args=None):
    """Forward ROS arguments to the shared capability entry point."""
    from peach_target_reconstruction.target_reconstruction_node import main as reconstruction_main

    return reconstruction_main(args=args)
