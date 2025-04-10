from collections import deque
from threading import Thread, Lock, Event
import select

class LogReader(Thread):

    def __init__(self, stream, depth):
        assert depth > 0
        self._stream = stream
        self._lines = deque(maxlen=depth)
        self._dirty = False
        self.mutex = Lock()
        self.interrupted = Event()

    def run(self):
        while True:
            try:
                readable, _, _ = select.select([self._stream], [], [], timeout=1)
                if self.interrupted.is_set():
                    break
                if readable == []:
                    continue

                    line = self._stream.readline()
                if not line or self.interrupted.is_set():
                    break
            except:
                break

            with self.mutex:
                self._lines.append(line.strip())
                self._dirty = True

    @property
    def is_dirty(self):
        return self._dirty

    def get_logs(self):
        with self.mutex:
            logs = list(self._lines)
            self._dirty = False

        return logs

    def interrupt(self):
        self.interrupted.set()