"""
HarvestDataStore：manifest/事件流/掩膜节流 + events.jsonl 并发追加锁.

首用例自 peach_pose_ros2/test/test_node_contract.py 迁移（行为不变）；
并发用例守门 A1 修复：双写者（模拟感知+重建双进程 attach 同一
run_dir）并发追加 events.jsonl 不出现行撕裂。
"""
import json
import threading
import unittest

import numpy as np

from peach_core.harvest_data import HarvestDataStore


class HarvestStoreContractTest(unittest.TestCase):
    """基本契约（迁移用例 + attach/query/节流）."""

    def test_harvest_store_manifest_events_and_mask(self):
        """一个 run 可关联 manifest、来源事件和时间戳掩膜（迁移用例）."""
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            store = HarvestDataStore(root=Path(tmp))
            run_dir = store.start('run_1', {'target_count': 1})
            store.append_event({'source': 'perception', 'event': 'locked'})
            mask_path = store.save_mask(
                'target_0', 123, np.ones((4, 5), dtype=np.uint8))
            assert (run_dir / 'manifest.yaml').is_file()
            assert (run_dir / 'events.jsonl').is_file()
            assert (run_dir / 'latest_perception.json').is_file()
            assert (run_dir / mask_path).is_file()

    def test_attach_existing_run_dir(self):
        """附着到既有目录（重建进程追加同一事件链）；缺失返回 False."""
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            owner = HarvestDataStore(root=Path(tmp))
            owner.start('run_1', {})
            guest = HarvestDataStore(root=Path(tmp))
            self.assertFalse(guest.attach('no_such_run'))
            self.assertTrue(guest.attach('run_1'))
            guest.append_event({'source': 'reconstruction', 'event': 'done'})
            lines = (owner.run_dir / 'events.jsonl').read_text(
                encoding='utf-8').splitlines()
            self.assertEqual(len(lines), 1)
            record = json.loads(lines[0])
            self.assertEqual(record['source'], 'reconstruction')

    def test_append_without_start_is_noop(self):
        """未 start/attach 时 append_event 静默丢弃（契约）."""
        store = HarvestDataStore(root='/nonexistent_root')
        store.append_event({'event': 'ignored'})   # 不抛异常
        self.assertEqual(store.query()['run_dir'], '')

    def test_save_mask_throttled_by_interval(self):
        """同一目标 min_interval_s 内重复保存返回 ''（节流）."""
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            store = HarvestDataStore(root=Path(tmp))
            store.start('run_1', {})
            mask = np.ones((4, 5), dtype=np.uint8)
            first = store.save_mask('t0', 1, mask, min_interval_s=60.0)
            second = store.save_mask('t0', 2, mask, min_interval_s=60.0)
            self.assertTrue(first)
            self.assertEqual(second, '')

    def test_query_returns_latest_event(self):
        """查询当前运行路径与最后事件（供状态话题复用）."""
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            store = HarvestDataStore(root=Path(tmp))
            store.start('run_1', {})
            store.append_event({'event': 'e1'})
            result = store.query()
            self.assertTrue(result['run_dir'].endswith('run_1'))
            self.assertEqual(result['latest']['event'], 'e1')


class HarvestStoreConcurrencyTest(unittest.TestCase):
    """events.jsonl flock 守门：双写者并发追加无行撕裂."""

    def test_concurrent_appends_produce_intact_lines(self):
        """两个 store（各自独立 fd）多线程并发追加，全部行均为合法 JSON."""
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            owner = HarvestDataStore(root=Path(tmp))
            run_dir = owner.start('run_1', {})
            guest = HarvestDataStore(root=Path(tmp))
            guest.attach('run_1')
            per_writer = 50

            def write_events(store, source):
                for i in range(per_writer):
                    store.append_event(
                        {'source': source, 'seq': i, 'pad': 'x' * 64})

            threads = [
                threading.Thread(
                    target=write_events, args=(owner, 'perception')),
                threading.Thread(
                    target=write_events, args=(guest, 'reconstruction')),
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()

            lines = (run_dir / 'events.jsonl').read_text(
                encoding='utf-8').splitlines()
            self.assertEqual(len(lines), 2 * per_writer)
            for line in lines:
                record = json.loads(line)   # 行撕裂会在这里抛 JSONDecodeError
                self.assertIn(record['source'],
                              ('perception', 'reconstruction'))


if __name__ == '__main__':
    unittest.main()
