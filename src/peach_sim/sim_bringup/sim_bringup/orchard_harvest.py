"""Compatibility entry: delegate to the cancelable simulation mission action client."""


def main(args=None):
    """Submit a mission through sim_harvest instead of commanding motion directly."""
    from sim_harvest.run_harvest import main as run
    return run(args)
