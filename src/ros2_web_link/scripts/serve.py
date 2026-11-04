#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Serve static views/config/resources; ROS traffic uses upstream rosbridge."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import math
from pathlib import Path
import re
import signal
from urllib.parse import quote, unquote, urlsplit

from ament_index_python.packages import get_package_share_directory
import yaml


RESOURCE_SUFFIXES = {
    '.urdf', '.stl', '.dae', '.obj', '.mtl', '.gltf', '.glb', '.bin', '.ply',
    '.png', '.jpg', '.jpeg', '.webp', '.gif', '.bmp', '.tga', '.tif', '.tiff',
    '.svg', '.dds', '.ktx', '.ktx2', '.exr', '.hdr',
}
SCENE_KEYS = {'fixedFrame', 'robotDescription', 'jointStates', 'tf', 'tfStatic',
              'markers', 'pointCloud', 'path'}
VIEW_KEYS = {'scene', 'image', 'depthImage', 'plot', 'diagnostics', 'rosout', 'project',
             'transportLimits', 'maxPendingRequests'}


def _read_profile(path, profile):
    """Load a selected profile with safe YAML, never Python object tags."""
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or set(document) != {'profiles'}:
        raise ValueError(f'{path}: expected a top-level profiles mapping')
    profiles = document['profiles']
    if not isinstance(profiles, dict) or set(profiles) - {'demo', 'robot'}:
        raise ValueError(f'{path}: profiles must contain demo and/or robot')
    selected = profiles.get(profile, {})
    if not isinstance(selected, dict) or set(selected) - VIEW_KEYS:
        raise ValueError(f'{path}: invalid view keys in profile {profile}')
    return selected


def _strings(mapping, required, optional=()):
    """Validate the flat string portions of the public configuration."""
    if not isinstance(mapping, dict) or not required <= set(mapping):
        raise ValueError(f'Expected mapping containing {sorted(required)}')
    unknown = set(mapping) - required - set(optional)
    if unknown:
        raise ValueError(f'Unknown view fields: {sorted(unknown)}')
    if any(not isinstance(mapping[key], str) or not mapping[key].strip()
           for key in required):
        raise ValueError(f'Expected nonempty strings for {sorted(required)}')


def _topic_options(options):
    """Validate the SDK TopicOptions shape without inventing a ROS transport."""
    if not isinstance(options, dict) or set(options) - {'qos', 'compression', 'throttleMs'}:
        raise ValueError('Subscription expects qos/compression/throttleMs')
    if options.get('compression', 'none') not in ('none', 'cbor'):
        raise ValueError('Subscription compression must be none or cbor')
    rate = options.get('throttleMs', 0)
    if isinstance(rate, bool) or not isinstance(rate, int) or rate < 0 or rate > 9007199254740991:
        raise ValueError('Subscription throttleMs must be a nonnegative safe integer')
    qos = options.get('qos', 'reliable')
    if isinstance(qos, str):
        if qos not in ('reliable', 'sensor', 'latched', 'staticTf'):
            raise ValueError('Unknown subscription QoS profile')
        return
    if (not isinstance(qos, dict)
            or set(qos) != {'history', 'depth', 'reliability', 'durability'}
            or qos['history'] != 'keep_last'
            or qos['reliability'] not in ('reliable', 'best_effort')
            or qos['durability'] not in ('volatile', 'transient_local')
            or isinstance(qos['depth'], bool) or not isinstance(qos['depth'], int)
            or not 1 <= qos['depth'] <= 9007199254740991):
        raise ValueError('Invalid explicit subscription QoS')


def load_config(default_path, custom_path, profile, ws_port, asset_packages,
                project='', projects_config=None):
    """Merge local view overrides while keeping CLI network/access authority."""
    view = _read_profile(default_path, profile)
    if custom_path:
        for key, value in _read_profile(custom_path, profile).items():
            if isinstance(value, dict) and isinstance(view.get(key), dict):
                view[key] = {**view[key], **value}
            else:
                view[key] = value
    if 'transportLimits' in view and not isinstance(view['transportLimits'], dict):
        raise ValueError('transportLimits must be a mapping; SDK validates individual budgets')
    if 'maxPendingRequests' in view:
        limit = view['maxPendingRequests']
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 4096:
            raise ValueError('maxPendingRequests must be an integer from 1 to 4096')
    _strings(view.get('scene'), SCENE_KEYS,
             {'pointCloudOptions', 'jointStatesOptions', 'robotDescriptionOptions',
              'markersOptions', 'pathOptions', 'profileShaders',
              'profileInteractions', 'shaderDiagnostics', 'useSimTime'})
    for key in ('pointCloudOptions', 'jointStatesOptions', 'robotDescriptionOptions',
                'markersOptions', 'pathOptions'):
        if key in view['scene']:
            _topic_options(view['scene'][key])
    for flag in ('profileShaders', 'profileInteractions', 'useSimTime'):
        if flag in view['scene'] and not isinstance(view['scene'][flag], bool):
            raise ValueError(f'scene.{flag} must be boolean')
    if view['scene'].get('shaderDiagnostics', 'errors-only') not in ('errors-only', 'verbose'):
        raise ValueError('scene.shaderDiagnostics must be errors-only or verbose')
    for key in ('image', 'depthImage'):
        image = view.get(key)
        if key == 'depthImage' and image is None:
            view.pop(key, None)
            continue
        _strings(image, {'topic', 'type'}, {'depthScale', 'subscription'})
        if 'subscription' in image:
            _topic_options(image['subscription'])
        if 'depthScale' in image:
            scale = image['depthScale']
            if (isinstance(scale, bool) or not isinstance(scale, (int, float))
                    or not math.isfinite(scale) or scale <= 0):
                raise ValueError(f'{key}.depthScale must be finite and positive')
    _strings(view.get('plot'), {'topic', 'type'}, {'fields', 'subscription'})
    if 'subscription' in view['plot']:
        _topic_options(view['plot']['subscription'])
    fields = view['plot'].get('fields')
    if (not isinstance(fields, list) or not fields
            or any(not isinstance(field, str) or not field for field in fields)):
        raise ValueError('plot.fields must be a nonempty string list')
    _strings({key: view.get(key) for key in ('diagnostics', 'rosout')},
             {'diagnostics', 'rosout'})
    # Project contracts are data only. The browser validates individual cases;
    # ROS-side command allowlists remain the launch operator's authority.
    projects = {}
    paths = [default_path.parent / 'projects.yaml']
    if projects_config:
        paths.append(projects_config)
    for path in paths:
        with path.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or set(document) != {'projects'}:
            raise ValueError(f'{path}: expected a projects mapping')
        incoming = document['projects']
        if not isinstance(incoming, dict):
            raise ValueError(f'{path}: projects must be a mapping')
        for key, definition in incoming.items():
            if (not isinstance(key, str)
                    or re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_-]{0,63}', key) is None
                    or not isinstance(definition, dict)
                    or definition.get('id') != key):
                raise ValueError(f'{path}: invalid project ID or definition')
            projects[key] = definition
    selected = project or view.pop('project', '') or (
        'demo' if profile == 'demo' else 'manual')
    if selected not in projects:
        raise ValueError(f'Unknown project: {selected}')
    if len(json.dumps(projects, allow_nan=False)) > 1000000:
        raise ValueError('Project configuration exceeds the 1 MB budget')
    return {**view, 'wsPort': ws_port, 'profile': profile,
            'assetPackages': asset_packages, 'project': selected,
            'projects': list(projects.values())}


class Handler(SimpleHTTPRequestHandler):
    """Constrain file access to the web root or explicitly allowed ROS shares."""

    extensions_map = {**SimpleHTTPRequestHandler.extensions_map,
                      '.urdf': 'application/xml', '.dae': 'model/vnd.collada+xml',
                      '.gltf': 'model/gltf+json', '.glb': 'model/gltf-binary',
                      '.stl': 'model/stl', '.obj': 'model/obj',
                      '.mjs': 'text/javascript'}

    def __init__(self, *args, config, asset_roots, **kwargs):
        self.config = config
        self.asset_roots = asset_roots
        super().__init__(*args, **kwargs)

    def list_directory(self, path):
        """Never expose directory inventories."""
        self.send_error(403, 'Directory listing disabled')

    def send_head(self):
        """Apply identical configuration and confinement rules to GET/HEAD."""
        try:
            url = urlsplit(self.path)
            path = unquote(url.path, errors='strict')
        except (UnicodeError, ValueError):
            self.send_error(400, 'Invalid URL')
            return None
        parts = path.split('/')
        if (url.scheme or url.netloc or not path.startswith('/')
                or '\\' in path or any(ord(char) < 32 for char in path)
                or any(part in ('.', '..') for part in parts)):
            self.send_error(403, 'Invalid resource path')
            return None
        if path == '/config.json':
            body = json.dumps(self.config, allow_nan=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            return BytesIO(body)
        if parts[1] == 'assets':
            if len(parts) < 4 or any(not part for part in parts[2:]):
                self.send_error(403, 'Expected /assets/<package>/<relative resource>')
                return None
            root = self.asset_roots.get(parts[2])
            if root is None:
                self.send_error(403, 'Package is not allowed')
                return None
            candidate = root.joinpath(*parts[3:])
            if candidate.suffix.lower() not in RESOURCE_SUFFIXES:
                self.send_error(403, 'Resource type is not allowed')
                return None
        else:
            root = Path(self.directory).resolve()
            candidate = root.joinpath(*parts[1:])
        try:
            candidate = candidate.resolve(strict=True)
            if not candidate.is_relative_to(root):
                self.send_error(403, 'Resource is outside its root')
                return None
            if parts[1] == 'assets' and candidate.suffix.lower() not in RESOURCE_SUFFIXES:
                self.send_error(403, 'Resource type is not allowed')
                return None
            if candidate.is_dir():
                if parts[1] == 'assets':
                    return self.list_directory(candidate)
                if not path.endswith('/'):
                    self.send_response(301)
                    self.send_header('Location', quote(path, safe='/') + '/')
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return None
                candidate = (candidate / 'index.html').resolve(strict=True)
                if not candidate.is_relative_to(root):
                    self.send_error(403, 'Resource is outside its root')
                    return None
            if not candidate.is_file():
                self.send_error(404, 'File not found')
                return None
            stream = candidate.open('rb')
        except (OSError, RuntimeError, ValueError):
            self.send_error(404, 'File not found')
            return None
        try:
            self.send_response(200)
            self.send_header('Content-Type', self.guess_type(str(candidate)))
            self.send_header('Content-Length', str(candidate.stat().st_size))
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
        except Exception:
            stream.close()
            raise
        return stream


def _port(value):
    port = int(value)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError('Port must be between 1 and 65535')
    return port


def _stop(_signum, _frame):
    raise KeyboardInterrupt


def main():
    """Close on launch shutdown; installed views need no Node.js runtime."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--port', type=_port, default=8088)
    parser.add_argument('--ws-port', type=_port, default=9090)
    parser.add_argument('--directory', type=Path)
    parser.add_argument('--profile', choices=('demo', 'robot'), default='robot')
    parser.add_argument('--config', default='')
    parser.add_argument('--project', default='')
    parser.add_argument('--projects-config', default='')
    parser.add_argument('--asset-packages', default='ros2_web_link')
    args, _ = parser.parse_known_args()
    share = Path(get_package_share_directory('ros2_web_link'))
    root = (args.directory or share / 'dist' / 'web').resolve()
    if not (root / 'index.html').is_file():
        parser.error('Web assets missing: run npm ci && npm run build in '
                     'src/ros2_web_link, then colcon build --packages-select ros2_web_link')
    packages = list(dict.fromkeys(
        name.strip() for name in args.asset_packages.split(',') if name.strip()))
    try:
        if any(re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]+', name) is None
               for name in packages):
            raise ValueError('asset_packages must contain ROS package names')
        asset_roots = {name: Path(get_package_share_directory(name)).resolve(strict=True)
                       for name in packages}
        config = load_config(share / 'config' / 'profiles.yaml',
                             Path(args.config) if args.config else None,
                             args.profile, args.ws_port, packages,
                             args.project, Path(args.projects_config)
                             if args.projects_config else None)
    except (OSError, LookupError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    signal.signal(signal.SIGTERM, _stop)
    handler = partial(Handler, directory=str(root), config=config, asset_roots=asset_roots)
    with ThreadingHTTPServer((args.address, args.port), handler) as server:
        print(f'ros2_web_link ({args.profile}): http://{args.address}:{args.port}', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
