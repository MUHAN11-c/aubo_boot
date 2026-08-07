"""SessionStore：会话三文件生成与内容一致；disabled 不落盘."""
import json
from pathlib import Path
import tempfile
import unittest

from peach_motion.data_store import SessionStore


class SessionStoreTest(unittest.TestCase):
    def test_session_three_files_and_content(self):
        """start_session 建目录三文件齐备；JSONL 逐行回读一致；summary 一致."""
        with tempfile.TemporaryDirectory() as tmp:
            store = SessionStore(root=str(Path(tmp) / 'peach_records'))
            session = store.start_session(stamp=1700000000.0)
            self.assertIsNotNone(session)
            targets = session / 'targets.jsonl'
            events = session / 'events.jsonl'
            self.assertTrue(targets.is_file())
            self.assertTrue(events.is_file())

            store.log_target({'target_id': 'T001', 'status': 'READY'})
            store.log_target({'target_id': 'T002', 'status': 'NEW'})
            store.log_event({'type': 'candidates', 'n': 2, '备注': '中文'})
            store.write_summary({'total': 2, 'done': 1})

            rows = [json.loads(line)
                    for line in targets.read_text(encoding='utf-8').splitlines()]
            self.assertEqual([r['target_id'] for r in rows], ['T001', 'T002'])
            self.assertEqual(rows[0]['status'], 'READY')
            ev = [json.loads(line)
                  for line in events.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(len(ev), 1)
            self.assertEqual(ev[0]['备注'], '中文')   # ensure_ascii=False 原样
            summary = json.loads(
                (session / 'summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary, {'total': 2, 'done': 1})

    def test_disabled_is_noop(self):
        """enabled=False：不建目录、不写任何文件."""
        with tempfile.TemporaryDirectory() as tmp:
            store = SessionStore(root=str(Path(tmp) / 'peach_records'),
                                 enabled=False)
            self.assertIsNone(store.start_session())
            store.log_target({'target_id': 'T001'})
            store.log_event({'type': 'x'})
            store.write_summary({'a': 1})
            self.assertFalse((Path(tmp) / 'peach_records').exists())

    def test_log_before_session_is_ignored(self):
        """未开会话先 log：静默忽略（不建文件）."""
        with tempfile.TemporaryDirectory() as tmp:
            store = SessionStore(root=str(Path(tmp) / 'peach_records'))
            store.log_target({'target_id': 'T001'})
            self.assertFalse((Path(tmp) / 'peach_records').exists())


if __name__ == '__main__':
    unittest.main()
