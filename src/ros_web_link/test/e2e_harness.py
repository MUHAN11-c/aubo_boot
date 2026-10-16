"""E2E 共享栈 harness — 桥+网关+fixture(+官方组件) 子进程编排（test_*_e2e*.py 复用）.

子进程经 asyncio.create_subprocess_exec + start_new_session 编排（Mimosa 约束下
的官方形态）；网络边界仅 127.0.0.1 回环，端口由内核分配；ROS_DOMAIN_ID 随机且
避开本机存活进程已占用的域（并发会话/长跑 demo 栈曾以 2/80/86 撞进随机范围，
表现为 /tf 混入他人帧与 JTC 目标被拒）。每个用例模块自行 importorskip 依赖，
本模块不做 skip 判定。

official=True 时额外起官方组件（2026-10-15 官方优先轮）：
  robot_state_publisher（MoveIt2 官方 panda URDF）+ `ros2 topic pub /joint_states`
  （官方 CLI 驱动）+ nav2_map_server + nav2_lifecycle_manager（官方地图）。
"""
from __future__ import annotations

import asyncio
import glob
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from random import SystemRandom

import httpx

_PKG_ROOT = Path(__file__).resolve().parent.parent
_WS_ROOT = _PKG_ROOT.parent.parent
_VENV_PY = _WS_ROOT / 'aubo_py3.12' / 'bin' / 'python'
_LOOPBACK = '127.0.0.1'
_HEALTH_PATH = '/health'
_READINESS_TIMEOUT_S = 30.0  # 任何单次等待 ≤30s（2026-10-15 用户约束）
_SRAND = SystemRandom()

# fixture 附加进程的模块名清单（Stack(fixtures=[...])）
# 支持 'module#args' 形态把参数传给该 fixture（如
# 'ros_web_link.comm_fixture#-p rwl.bigdata:=true'）；KNOWN_FIXTURES 只校验模块段。
KNOWN_FIXTURES = ('ros_web_link.comm_fixture',)

# 官方组件常量（固定绝对路径；robot_state_publisher 的 URDF 插件需要 vendor 库链：
# sdformat14→gz-math7、ompl —— 见 doc/architecture.md 已知坑）
_ROS2_BIN = '/opt/ros/jazzy/bin/ros2'
_XACRO_BIN = '/opt/ros/jazzy/bin/xacro'
_PANDA_XACRO = str(_WS_ROOT / 'install' / 'moveit_resources_panda_description'
                   / 'share' / 'moveit_resources_panda_description' / 'urdf'
                   / 'panda.urdf.xacro')
_OFFICIAL_MAP = '/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml'
_PANDA_PREFIX = str(_WS_ROOT / 'install' / 'moveit_resources_panda_description')
_VENDOR_LIBS = sorted(glob.glob('/opt/ros/jazzy/opt/*/lib')) + [
    '/opt/ros/jazzy/lib/x86_64-linux-gnu']

_DOMAIN_RE = re.compile(rb'^ROS_DOMAIN_ID=(\d+)$', re.M)

_PANDA_JOINTS = ('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                 'panda_joint5', 'panda_joint6', 'panda_joint7')
_PANDA_HOME = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)


def official_env(base: dict) -> dict:
    """官方组件 env：叠加 vendor 库链（RSP 的 sdformat 插件依赖）."""
    env = dict(base)
    env['LD_LIBRARY_PATH'] = ':'.join(
        _VENDOR_LIBS + [env.get('LD_LIBRARY_PATH', '')])
    return env


async def _panda_urdf_text(env: dict | None = None) -> str:
    """官方 panda URDF（官方 xacro 展开；mesh 为 file:// 绝对路径，web 端负责重写）.

    xacro 需在 AMENT_PREFIX_PATH 里看到 panda 包（pytest 环境不含工作区全部前缀）。
    """
    run_env = dict(env or os.environ)
    run_env['AMENT_PREFIX_PATH'] = _PANDA_PREFIX + os.pathsep \
        + run_env.get('AMENT_PREFIX_PATH', '')
    proc = await asyncio.create_subprocess_exec(
        _XACRO_BIN, _PANDA_XACRO,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
        env=run_env)
    out, _ = await asyncio.wait_for(proc.communicate(), 20.0)
    if proc.returncode != 0:
        raise RuntimeError(f'xacro 展开失败 rc={proc.returncode}')
    # 尾部换行必须去掉：bash $(...) 会吃掉它，而 rcl 的 -p name:=value
    # YAML 解析对带尾换行的多行值直接报 RCLInvalidROSArgsError（rc=250）
    return out.decode().rstrip('\n')


def _rsp_params_file(urdf: str) -> str:
    """RSP 参数文件（YAML block scalar 装多行 URDF）.

    多行 URDF 经 `-p name:=<value>` 直传在 execve 路径下会被 rcl 参数 YAML
    解析拒绝（RCLInvalidROSArgsError rc=250；bash $(...) 路径却可过——传递层
    差异未深究）。--params-file 是 launch 传大 URDF 的官方标准机制，稳定。
    """
    import tempfile
    import textwrap
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='rwl_rsp_params_')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write('robot_state_publisher:\n  ros__parameters:\n'
                '    robot_description: |\n'
                + textwrap.indent(urdf, '      ') + '\n')
    return path


def busy_domain_ids() -> set[int]:
    """本机存活进程正在使用的 ROS_DOMAIN_ID 集合（读 /proc/*/environ，不碰 DDS）.

    并发会话的 demo 栈（panda/peach）常驻域 2/80/86 一类值；随机域撞上它们
    会出现 /tf 混入他人帧、action 目标被拒等跨栈串扰。
    """
    out: set[int] = set()
    try:
        entries = list(Path('/proc').iterdir())
    except OSError:
        return out
    for p in entries:
        if not p.name.isdigit():
            continue
        try:
            env = (p / 'environ').read_bytes()
        except (OSError, PermissionError):
            continue
        for m in _DOMAIN_RE.finditer(env):
            out.add(int(m.group(1)))
    return out


def free_domain(lo: int = 2, hi: int = 230) -> int:
    """随机挑一个未被本机进程占用的域（避开 busy 集合；全忙时退回纯随机）."""
    busy = busy_domain_ids()
    for _ in range(64):
        d = _SRAND.randint(lo, hi)
        if d not in busy:
            return d
    return _SRAND.randint(lo, hi)


def stack_python() -> str:
    """子进程解释器：优先当前解释器，缺 websockets 时回退工作区 venv."""
    try:
        import websockets  # noqa: F401
        return sys.executable
    except ImportError:
        pass
    if _VENV_PY.is_file():
        return str(_VENV_PY)
    raise RuntimeError('当前解释器缺 websockets 且未找到 aubo_py3.12 venv')


def bridge_executable() -> Path:
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ivg_foxglove_bridge'))
    exe = share.parent.parent / 'lib' / 'ivg_foxglove_bridge' / 'foxglove_bridge'
    if not exe.is_file():
        raise FileNotFoundError(f'未找到桥可执行文件: {exe}')
    return exe


def free_port() -> int:
    """由内核分配空闲回环端口（避开临时端口段 32768+，防后续出站连接抢占导致
    网关/桥 bind 失败——2026-10-14 panda E2E 实证：uvicorn bind 被抢后静默
    respawn 死循环，表象为 gateway 永不就绪）."""
    for _ in range(64):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((_LOOPBACK, 0))
            port = s.getsockname()[1]
            if port < 30000:
                return port
    return port


def loopback_client(port: int) -> httpx.Client:
    """回环专用 httpx 客户端：base_url 绑死 127.0.0.1，禁重定向."""
    return httpx.Client(
        base_url='http://' + _LOOPBACK + ':' + str(port) + '/',
        timeout=15.0,
        follow_redirects=False,
        trust_env=False,
    )


class Stack:
    """子进程编排：桥 → 网关 → 官方组件（可选）→ fixtures；asyncio 子进程 + 常驻后台事件循环."""

    def __init__(self, fixtures: tuple[str, ...] = (), official: bool = False) -> None:
        for f in fixtures:
            if f.partition('#')[0] not in KNOWN_FIXTURES:
                raise ValueError(f'未知 fixture: {f}（已知 {KNOWN_FIXTURES}）')
        self.fixtures = fixtures
        self.official = official
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._children: list = []
        self.bridge_port = free_port()
        self.gateway_port = free_port()
        self._bridge_port_s = str(self.bridge_port)
        self._gateway_port_s = str(self.gateway_port)
        self._domain_s = str(free_domain())
        self._bridge_port_arg = 'port:=' + self._bridge_port_s
        self._bridge_addr_arg = 'address:=' + _LOOPBACK
        self._bridge_caps_arg = 'capabilities:=[clientPublish,services,connectionGraph]'
        self._gateway_ws_url = 'ws://' + _LOOPBACK + ':' + self._gateway_port_s + '/ws/foxglove'
        self._pkg_root_s = str(_PKG_ROOT)
        self._pythonpath = self._pkg_root_s + os.pathsep + os.environ.get('PYTHONPATH', '')

    def _env(self) -> dict:
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = self._domain_s
        env.setdefault('ROS_DISTRO', 'jazzy')
        env['PYTHONPATH'] = self._pythonpath
        bridge_lib = str(bridge_executable().parent.parent)
        env['LD_LIBRARY_PATH'] = bridge_lib + os.pathsep + env.get('LD_LIBRARY_PATH', '')
        return env

    def _official_procs(self, env: dict) -> list[tuple[str, list]]:
        """官方组件进程表（(标签, argv)）；panda 资源缺失时抛 RuntimeError."""
        if not Path(_PANDA_XACRO).is_file():
            raise RuntimeError(
                f'官方 panda 资源缺失: {_PANDA_XACRO}（official 栈需 moveit_resources_panda）')
        return [
            ('rsp', [_ROS2_BIN, 'run', 'robot_state_publisher', 'robot_state_publisher',
                     '--ros-args', '--params-file', self._rsp_params_path,
                     '-p', 'use_sim_time:=false']),
            ('joint_pub', [_ROS2_BIN, 'topic', 'pub', '-r', '1', '/joint_states',
                           'sensor_msgs/msg/JointState',
                           '{name: [' + ', '.join(_PANDA_JOINTS) + '],'
                           ' position: [' + ', '.join(str(v) for v in _PANDA_HOME) + ']}']),
            ('map_server', [_ROS2_BIN, 'run', 'nav2_map_server', 'map_server',
                            '--ros-args', '-p', 'yaml_filename:=' + _OFFICIAL_MAP,
                            '-p', 'use_sim_time:=false']),
            ('lifecycle', [_ROS2_BIN, 'run', 'nav2_lifecycle_manager',
                           'lifecycle_manager',
                           '--ros-args', '-p', 'use_sim_time:=false',
                           '-p', 'node_names:=[map_server]']),
        ]

    def start(self) -> None:
        py = stack_python()
        env = self._env()
        bridge_cmd = [
            str(bridge_executable()), '--ros-args',
            '-p', self._bridge_port_arg,
            '-p', self._bridge_addr_arg,
            '-p', self._bridge_caps_arg,
        ]
        gateway_cmd = [
            py, '-m', 'ros_web_link.cli', self._gateway_port_s,
            '--bind', _LOOPBACK,
            '--foxglove-bridge-port', self._bridge_port_s,
        ]
        # (argv, env) 对：官方组件需要 vendor 库链 env
        procs: list[tuple[list[str], dict]] = [(bridge_cmd, env), (gateway_cmd, env)]
        if self.official:
            self._panda_urdf = asyncio.run(_panda_urdf_text(env))
            self._rsp_params_path = _rsp_params_file(self._panda_urdf)
            for _, cmd in self._official_procs(env):
                procs.append((cmd, official_env(env)))
        for f in self.fixtures:
            mod, _, extra = f.partition('#')
            cmd = [py, '-m', mod] + (extra.split() if extra else [])
            procs.append((cmd, env))

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, name='rwl-e2e-procs', daemon=True)
        self._thread.start()

        async def spawn():
            for cmd, env_ in procs:
                self._children.append(await asyncio.create_subprocess_exec(
                    *cmd, env=env_, start_new_session=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT))

        asyncio.run_coroutine_threadsafe(spawn(), self._loop).result(timeout=30.0)
        for p in self._children:
            if p.returncode is not None:
                self.stop()
                raise RuntimeError('子进程启动即退出 (pid=' + str(p.pid) + ')')
        self._wait_gateway()

    def ensure_map_active(self, timeout_s: float = 25.0) -> bool:
        """等 /map_server 到 active（lifecycle_manager 自动发现偶发卡死时官方 CLI 兜底）.

        经栈自带事件循环跑官方 `ros2 lifecycle get/set`（与栈同域 env）。
        """
        env = official_env(self._env())

        async def run_cli(*args: str) -> str:
            proc = await asyncio.create_subprocess_exec(
                _ROS2_BIN, *args,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
                env=env)
            out, _ = await asyncio.wait_for(proc.communicate(), 10.0)
            return out.decode().strip()

        async def wait_active() -> bool:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                state = await run_cli('lifecycle', 'get', '/map_server')
                if state.startswith('active'):
                    return True
                if state.startswith('unconfigured'):
                    await run_cli('lifecycle', 'set', '/map_server', 'configure')
                elif state.startswith('inactive'):
                    await run_cli('lifecycle', 'set', '/map_server', 'activate')
                await asyncio.sleep(0.8)
            return False

        loop = self._loop
        if loop is None:
            return False
        return asyncio.run_coroutine_threadsafe(wait_active(), loop).result(
            timeout=timeout_s + 5.0)

    def _wait_gateway(self) -> None:
        deadline = time.monotonic() + _READINESS_TIMEOUT_S
        last_err = None
        while time.monotonic() < deadline:
            try:
                with loopback_client(self.gateway_port) as client:
                    r = client.get(_HEALTH_PATH)
                    if r.status_code == 200:
                        return
            except Exception as exc:  # noqa: BLE001
                last_err = exc
            time.sleep(0.3)
        self.stop()
        raise RuntimeError('gateway 未就绪: ' + repr(last_err))

    def stop(self) -> None:
        loop, self._loop = self._loop, None
        thread = self._thread
        self._thread = None
        if loop is not None and thread is not None:
            children = self._children

            async def shutdown():
                for p in children:
                    if p.returncode is None:
                        try:
                            os.killpg(p.pid, signal.SIGINT)
                        except (ProcessLookupError, PermissionError):
                            p.terminate()
                for p in children:
                    if p.returncode is None:
                        try:
                            await asyncio.wait_for(p.wait(), 8.0)
                        except asyncio.TimeoutError:
                            try:
                                os.killpg(p.pid, signal.SIGKILL)
                            except (ProcessLookupError, PermissionError):
                                p.kill()
                            await p.wait()
            try:
                asyncio.run_coroutine_threadsafe(
                    shutdown(), loop).result(timeout=25.0)
            finally:
                loop.call_soon_threadsafe(loop.stop)
                thread.join(timeout=5.0)
                loop.close()
        left = [p.pid for p in self._children if p.returncode is None]
        assert not left, '清理后仍有存活子进程: ' + str(left)
        self._children.clear()
        for port in (self.gateway_port, self.bridge_port):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                try:
                    s.connect((_LOOPBACK, port))
                except OSError:
                    continue
                raise AssertionError(f'端口 {port} 仍有监听者，子进程未停干净')

    @property
    def ws_url(self) -> str:
        return self._gateway_ws_url


class LaunchStack:
    """整仓 launch 子进程编排：ros2 launch <pkg> <file>（sim_demo 等官方组合栈）.

    与 Stack（直编桥/网关/官方进程）不同，这里整仓 launch 一次拉起（栈内部自己
    组合官方组件）；环境补齐 vendor 库链与工作区前缀，域名/端口随机避让。
    """

    def __init__(self, launch_file: str, args: dict | None = None,
                 port_arg: str = 'port', gateway_arg: str = 'gateway_port') -> None:
        self.launch_file = launch_file
        self.args = args or {}
        self._port_arg = port_arg
        self._gateway_arg = gateway_arg
        self.gateway_port = free_port()
        self.bridge_port = free_port()
        self._domain_s = str(free_domain())
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._proc = None
        self._log_path = None
        self._keep_log = False
        self._last_have: set[str] = set()

    def _env(self) -> dict:
        env = os.environ.copy()
        ws_installs = [
            'ros_web_link', 'ivg_foxglove_bridge', 'moveit_resources_panda_description',
            'turtlebot3_gazebo', 'turtlebot3_msgs', 'turtlebot3_navigation2',
            'examples_rclpy_pointcloud_publisher', 'examples_rclpy_minimal_publisher',
        ]
        env.update({
            'ROS_DOMAIN_ID': self._domain_s,
            'ROS_DISTRO': 'jazzy',
            'TURTLEBOT3_MODEL': 'waffle_pi',
            'AMENT_PREFIX_PATH': ':'.join(
                [str(_WS_ROOT / 'install' / p) for p in ws_installs] + ['/opt/ros/jazzy']),
            'PYTHONPATH': os.pathsep.join(
                ['/opt/ros/jazzy/lib/python3.12/site-packages', str(_PKG_ROOT)]
                + [str(_WS_ROOT / 'install' / p
                   / 'lib' / 'python3.12' / 'site-packages')
                   for p in ws_installs]
                # ament_python symlink-install 是 egg-link 开发安装：
                # importlib.metadata 需要源/build 目录（含 egg-info）直接在
                # sys.path 上（PYTHONPATH 条目不处理 .pth）
                + [str(_WS_ROOT / 'build' / p) for p in (
                    'examples_rclpy_pointcloud_publisher',
                    'examples_rclpy_minimal_publisher')]),
            'LD_LIBRARY_PATH': ':'.join(
                sorted(glob.glob('/opt/ros/jazzy/opt/*/lib'))
                + ['/opt/ros/jazzy/lib', '/opt/ros/jazzy/lib/x86_64-linux-gnu',
                   str(_WS_ROOT / 'install' / 'ivg_foxglove_bridge' / 'lib')]),
            # gz 工具链（source setup.bash 的等价最小集）
            'GZ_CONFIG_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/share/gz',
            'PATH': ':'.join([
                str(_VENV_PY.parent), '/opt/ros/jazzy/opt/gz_tools_vendor/bin',
                '/opt/ros/jazzy/bin', '/usr/bin', '/bin']),
        })
        return env

    def start(self, readiness_topics: tuple[str, ...] = (),
              readiness_budget_s: float = 150.0) -> None:
        import subprocess
        import tempfile
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, name='rwl-launch-stack', daemon=True)
        self._thread.start()

        async def spawn():
            fh = tempfile.NamedTemporaryFile(
                prefix='rwl_launch_', suffix='.log', delete=False)
            self._log_path = fh.name
            argv = [_ROS2_BIN, 'launch', 'ros_web_link', self.launch_file,
                    f'{self._gateway_arg}:={self.gateway_port}',
                    f'{self._port_arg}:={self.bridge_port}',
                    # gz-transport 分区隔离：本机其他 gz 仿真（如 peach 果园）
                    # 常驻默认分区，不隔离会交叉发布 /odom /scan（实证串扰）
                    f'gz_partition:=rwl_{self._domain_s}']
            for k, v in self.args.items():
                argv.append(f'{k}:={v}')
            self._proc = await asyncio.create_subprocess_exec(
                *argv, env=self._env(), start_new_session=True,
                stdout=fh, stderr=subprocess.STDOUT)

        asyncio.run_coroutine_threadsafe(spawn(), self._loop).result(timeout=30.0)
        self._wait_gateway()
        if readiness_topics:
            if not self._wait_topics(readiness_topics, readiness_budget_s):
                missing = sorted(set(readiness_topics) - self._last_have)
                tail = ''
                if self._log_path and os.path.exists(self._log_path):
                    try:
                        with open(self._log_path, encoding='utf-8',
                                  errors='replace') as f:
                            tail = f.read()[-1500:]
                    except OSError:
                        pass
                self.stop()
                raise RuntimeError(
                    f'官方栈就绪超时（{readiness_budget_s}s）：缺 {missing}；'
                    f'日志 {self._log_path}\n[launch 日志尾]\n{tail}')

    def _wait_gateway(self) -> None:
        deadline = time.monotonic() + _READINESS_TIMEOUT_S
        last = None
        while time.monotonic() < deadline:
            try:
                with loopback_client(self.gateway_port) as c:
                    if c.get(_HEALTH_PATH).status_code == 200:
                        return
            except Exception as exc:  # noqa: BLE001
                last = exc
            time.sleep(0.5)
        self._keep_log = True
        self.stop()
        tail = ''
        if self._log_path and os.path.exists(self._log_path):
            try:
                with open(self._log_path, encoding='utf-8', errors='replace') as f:
                    tail = f.read()[-2000:]
            except OSError:
                pass
        raise RuntimeError(f'gateway 未就绪 ({self._log_path}): ' + repr(last)
                           + '\n[launch 日志尾]\n' + tail)

    async def _run_cli(self, *argv: str, timeout_s: float = 20.0) -> tuple[int, str]:
        proc = await asyncio.create_subprocess_exec(
            _ROS2_BIN, *argv, env=self._env(),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        try:
            out, _ = await asyncio.wait_for(proc.communicate(), timeout_s)
            return proc.returncode, out.decode(errors='replace')
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
            return -1, ''

    def _wait_topics(self, topics: tuple[str, ...],
                     budget_s: float) -> bool:
        """轮询 ros2 topic list 直到全部出现（单次 CLI ≤20s，符合 30s 上限约束）."""

        async def run():
            deadline = time.monotonic() + budget_s
            last_have: set[str] = set()
            while time.monotonic() < deadline:
                rc, out = await self._run_cli('topic', 'list')
                if rc == 0:
                    last_have = {line.strip() for line in out.splitlines()
                                 if line.strip()}
                    if all(t in last_have for t in topics):
                        return True
                await asyncio.sleep(2.0)
            self._last_have = last_have
            return False

        ok = asyncio.run_coroutine_threadsafe(run(), self._loop).result(
            timeout=budget_s + 30.0)
        if not ok:
            self._keep_log = True
        return ok

    def stop(self) -> None:
        import signal
        loop, self._loop = self._loop, None
        thread, self._thread = self._thread, None
        proc = self._proc
        if loop is not None and thread is not None:
            async def shutdown():
                if proc is not None and proc.returncode is None:
                    try:
                        os.killpg(proc.pid, signal.SIGINT)
                    except (ProcessLookupError, PermissionError):
                        proc.terminate()
                    try:
                        await asyncio.wait_for(proc.wait(), 20.0)
                    except asyncio.TimeoutError:
                        try:
                            os.killpg(proc.pid, signal.SIGKILL)
                        except (ProcessLookupError, PermissionError):
                            proc.kill()
                        await proc.wait()
            try:
                asyncio.run_coroutine_threadsafe(
                    shutdown(), loop).result(timeout=30.0)
            finally:
                loop.call_soon_threadsafe(loop.stop)
                thread.join(timeout=5.0)
                loop.close()
                if self._log_path and os.path.exists(self._log_path) \
                        and not self._keep_log:
                    try:
                        os.unlink(self._log_path)
                    except OSError:
                        pass
        left = [p.pid for p in [self._proc] if p is not None and p.returncode is None]
        assert not left, '清理后仍有存活子进程: ' + str(left)

    @property
    def ws_url(self) -> str:
        return 'ws://' + _LOOPBACK + ':' + str(self.gateway_port) + '/ws/foxglove'


class Connected:
    """每个用例独立 WS 连接（干净隔离；advertise 列表随连接重放）."""

    def __init__(self, stack: Stack):
        self._url = stack.ws_url

    async def __aenter__(self):
        from ros_web_link.foxglove_ws_client import FoxgloveWsClient
        deadline = time.monotonic() + _READINESS_TIMEOUT_S
        while True:
            c = FoxgloveWsClient()
            try:
                await c.connect(self._url, timeout=5.0)
                self._c = c
                return c
            except Exception:
                await c.close()
                if time.monotonic() > deadline:
                    raise
                await asyncio.sleep(0.5)

    async def __aexit__(self, *exc) -> None:
        await self._c.close()
