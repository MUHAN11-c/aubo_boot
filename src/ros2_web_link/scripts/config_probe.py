#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Explicit real-YAML configuration checks, without an HTTP/ROS server."""

from pathlib import Path
import tempfile

from serve import load_config
import yaml


def main():
    """Exercise the same loader used by both deployed HTTP profiles."""
    default = Path(__file__).resolve().parents[1] / 'config/profiles.yaml'
    for profile in ('demo', 'robot'):
        config = load_config(default, None, profile, 9090, ['ros2_web_link'])
        assert config['scene']['useSimTime'] is False
        assert config['wsPort'] == 9090
    print('PASS default demo and robot YAML load with explicit system time')
    config = load_config(default, None, 'robot', 9090, [], project='peach_replay')
    projects = {project['id']: project for project in config['projects']}
    for name in ('peach_replay', 'peach_replay_large'):
        assert projects[name]['view']['scene']['useSimTime'] is True
    for name in ('demo', 'peach'):
        assert projects[name]['view']['scene']['useSimTime'] is False
    print('PASS recorded and live project definitions keep their selected clock mode')
    with tempfile.TemporaryDirectory(prefix='ros2-web-config-probe-') as directory:
        override = Path(directory) / 'views.yaml'
        for field in ('useSimTime', 'profileInteractions', 'profileShaders'):
            for value in (False, True):
                override.write_text(yaml.safe_dump({'profiles': {'demo': {'scene': {
                    field: value}}}}))
                config = load_config(default, override, 'demo', 9090, [])
                assert config['scene'][field] is value
            for value in ('false', 0, 1, [], {}):
                override.write_text(yaml.safe_dump({'profiles': {'demo': {'scene': {
                    field: value}}}}))
                try:
                    load_config(default, override, 'demo', 9090, [])
                except ValueError as error:
                    assert field in str(error) and 'boolean' in str(error)
                else:
                    raise AssertionError(f'{field} accepted non-boolean {value!r}')
    print('PASS all three boolean scene flags accept booleans and reject coercion')


if __name__ == '__main__':
    main()
