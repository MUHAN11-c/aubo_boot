# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""
系统/GPU/关键进程性能采样线程（只读监控用，零 ROS 依赖）.

独立守护线程周期采样，经回调写入 DashboardState，绝不阻塞 ROS 回调
线程；psutil 或 nvidia-smi 不可用时对应字段降级为 None/空表。
"""

from __future__ import annotations

import subprocess
import threading
import time

try:
    import psutil
except ImportError:  # pragma: no cover - 部署机保证有 psutil，仅防御
    psutil = None


class MetricsSampler:
    """周期采集 CPU/内存/load、GPU 与关键进程 CPU/RSS 的后台线程."""

    def __init__(self, period_s: float, patterns: list[str], apply,
                 log_warning=lambda msg: None):
        """保存采样周期、进程 cmdline 匹配关键字与结果回调."""
        self._period = max(0.5, float(period_s))
        self._patterns = [str(item) for item in patterns if str(item)]
        self._apply = apply
        self._log_warning = log_warning
        self._stop = threading.Event()
        self._thread = None
        # 按 pid 缓存 Process 句柄：cpu_percent(None) 依赖上次调用做差分
        self._processes = {}

    def start(self) -> None:
        """启动后台采样线程（幂等）."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name='peach-perception-metrics', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """置停止标志并等待线程退出."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._apply(self._collect())
            except Exception as error:  # 采样失败只降级，绝不炸采样线程
                self._log_warning(f'性能采样失败（本轮跳过）: {error}')
            self._stop.wait(self._period)

    def _collect(self) -> dict:
        """采集一帧性能样本；任何子项失败都局部降级."""
        sample = {
            'stamp': time.time(),
            'cpu_percent': None,
            'memory_percent': None,
            'memory_used_mb': None,
            'memory_total_mb': None,
            'load1': None,
            'load5': None,
            'load15': None,
            'gpu': self._collect_gpu(),
            'processes': self._collect_processes(),
        }
        if psutil is not None:
            try:
                sample['cpu_percent'] = float(psutil.cpu_percent(None))
                memory = psutil.virtual_memory()
                sample['memory_percent'] = float(memory.percent)
                sample['memory_used_mb'] = round(memory.used / 1048576.0, 1)
                sample['memory_total_mb'] = round(memory.total / 1048576.0, 1)
                load1, load5, load15 = psutil.getloadavg()
                sample['load1'] = round(float(load1), 2)
                sample['load5'] = round(float(load5), 2)
                sample['load15'] = round(float(load15), 2)
            except (OSError, RuntimeError) as error:
                self._log_warning(f'系统性能采样降级: {error}')
        return sample

    def _collect_gpu(self) -> dict | None:
        """nvidia-smi 查询首块 GPU；不可用/超时/解析失败一律降级 None."""
        try:
            result = subprocess.run(
                ['nvidia-smi',
                 '--query-gpu=utilization.gpu,memory.used,memory.total',
                 '--format=csv,noheader,nounits'],
                capture_output=True, text=True, timeout=2.0, check=False)
            if result.returncode != 0:
                return None
            first = result.stdout.strip().splitlines()[0]
            util, used, total = (float(part.strip()) for part in first.split(','))
            return {
                'utilization_percent': util,
                'memory_used_mb': used,
                'memory_total_mb': total,
            }
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            return None

    def _collect_processes(self) -> list[dict]:
        """按 cmdline 关键字匹配关键进程，取每组 RSS 最大者报 CPU/RSS."""
        if psutil is None or not self._patterns:
            return []
        try:
            candidates = list(psutil.process_iter(['pid', 'cmdline']))
        except (OSError, RuntimeError):
            return []
        matched = []
        live_pids = set()
        for pattern in self._patterns:
            best = None
            for info in candidates:
                cmdline = info.info.get('cmdline') or []
                if pattern not in ' '.join(cmdline):
                    continue
                try:
                    process = self._processes.get(info.info['pid'])
                    if process is None:
                        process = psutil.Process(info.info['pid'])
                        self._processes[info.info['pid']] = process
                    live_pids.add(process.pid)
                    rss = process.memory_info().rss
                    if best is None or rss > best[1]:
                        best = (process, rss)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            if best is None:
                continue
            process, rss = best
            try:
                matched.append({
                    'name': pattern,
                    'pid': process.pid,
                    'cpu_percent': float(process.cpu_percent(None)),
                    'rss_mb': round(rss / 1048576.0, 1),
                })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        # 清掉已退出进程的缓存句柄，防止 pid 复用后误读
        for pid in [pid for pid in self._processes if pid not in live_pids]:
            del self._processes[pid]
        return matched
