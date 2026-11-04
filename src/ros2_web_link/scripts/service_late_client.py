# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Keep up to three already-timed-out readers briefly, only for diagnosis."""

import atexit
import json
import os
from pathlib import Path
import sys
from threading import current_thread, Lock, Timer
import time


class LateClientTrace:
    """Change cleanup only after the original timeout verdict, never retry."""

    def __init__(self, root, original_destroy, hold_seconds=2.0):
        self._root, self._original = Path(root), original_destroy
        if not 0 < hold_seconds <= 2:
            raise ValueError('Late-reader diagnostic hold must be positive and <=2 seconds')
        self._seconds = hold_seconds
        self._guard = Lock()
        self._armed, self._records, self._timers = {}, [], set()
        self._errors = self._cancelled = 0
        self._budget = self._closing = self._final_attempted = False
        atexit.register(self.final_snapshot)

    def arm(self, client_id, cid):
        """Retain just a primitive ID until upstream requests client cleanup."""
        with self._guard:
            if not isinstance(client_id, int) or client_id <= 0:
                self._errors += 1
                return False
            if self._closing or len(self._records) >= 3:
                self._budget = True
                return False
            if client_id in self._armed:
                return False
            record = {'cid': cid[:128] if isinstance(cid, str) else None,
                      'armedAt': time.monotonic()}
            self._records.append(record)
            self._armed[client_id] = record
            return True

    def destroy(self, node, client):
        """Forward normal cleanup immediately; defer only an armed timeout."""
        with self._guard:
            record = self._armed.pop(id(client), None)
        if record is None:
            return self._original(node, client)
        record['destroyRequestedAt'] = time.monotonic()
        timer = None
        try:
            timer = Timer(self._seconds, self._submit, args=(node, client, record))
            timer.daemon = True
            with self._guard:
                self._timers.add(timer)
            timer.start()
        except Exception:
            if timer is not None:
                timer.cancel()
                if timer.ident is not None:
                    timer.join(timeout=.05)
            with self._guard:
                self._timers.discard(timer)
                self._errors += 1
                record['timerStartFailed'] = True
            # Failure to observe must not prevent original entity cleanup.
            return self._original(node, client)
        return None

    def _submit(self, node, client, record):
        """Use the original executor scheduler, never directly destroy a handle."""
        try:
            if not node.context.ok():
                raise RuntimeError('Context closed before deferred cleanup submission')
            self._original(node, client)
            record['cleanupSubmittedAt'] = time.monotonic()
        except Exception as error:
            with self._guard:
                self._errors += 1
                record['cleanupErrorType'] = type(error).__name__
        finally:
            # Timer owns the temporary arguments until its thread finishes.
            # Removing our reference lets node/client references go then too.
            with self._guard:
                self._timers.discard(current_thread())

    def final_snapshot(self):
        """Cancel unfinished timer threads and attempt one bounded exit record."""
        if self._final_attempted:
            return
        self._final_attempted = True
        try:
            acquired = self._guard.acquire(timeout=.05)
            try:
                self._closing = True
                timers = list(self._timers) if acquired else []
            finally:
                if acquired:
                    self._guard.release()
            for timer in timers:
                timer.cancel()
                if timer.ident is not None:
                    timer.join(timeout=.05)
                self._cancelled += 1
            acquired = self._guard.acquire(timeout=.05)
            try:
                record = {'holdSeconds': self._seconds, 'holdLimit': 3,
                          'records': [dict(r) for r in self._records] if acquired else [],
                          'snapshotAvailable': acquired, 'errors': self._errors,
                          'budgetExceeded': self._budget, 'cancelledOnExit': self._cancelled,
                          'armedPending': len(self._armed) if acquired else None,
                          'activeTimers': sum(t.is_alive() for t in timers),
                          'scope': 'post-timeout reader retention; original verdict unchanged'}
                if acquired:
                    self._timers.difference_update(t for t in timers if not t.is_alive())
            finally:
                if acquired:
                    self._guard.release()
            with (self._root / f'service-late-client-{os.getpid()}.json').open(
                    'x', encoding='utf-8') as output:
                json.dump(record, output)
                output.write('\n')
        except Exception as error:
            try:
                print(f'late-reader diagnostic write failed: {type(error).__name__}',
                      file=sys.stderr)
            except Exception:
                pass


def read_late_client_trace(root):
    """Reject incomplete post-timeout lifetime evidence after process cleanup."""
    paths = list(Path(root).glob('service-late-client-*.json'))
    if len(paths) != 1 or paths[0].stat().st_size > 8192:
        raise ValueError('Expected one bounded late-reader snapshot')
    record = json.loads(paths[0].read_text())
    incomplete = any('cleanupSubmittedAt' not in r for r in record['records'])
    if (not record['snapshotAvailable'] or record['errors'] or record['budgetExceeded'] or
            record['cancelledOnExit'] or record['armedPending'] or record['activeTimers'] or
            len(record['records']) > 3 or incomplete):
        raise ValueError('Late-reader cleanup evidence incomplete')
    return record
