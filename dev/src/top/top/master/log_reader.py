from collections import deque
from threading import Thread, Lock, Event
import select
import subprocess

class LogReader(Thread):

    def __init__(self, stream, depth):
        assert depth > 0
        super().__init__()
        self._stream = stream
        self._lines = deque(maxlen=depth)
        self._dirty = False
        self.mutex = Lock()
        self.interrupted = Event()

    def run(self):
        while True:
            try:
                readable, _, _ = select.select([self._stream], [], [], 1)
                if self.interrupted.is_set():
                    break
                if readable == []:
                    continue

                line = self._stream.readline()
                if not line or self.interrupted.is_set():
                    break
            except:
                # Stream closed (process ended)
                break

            with self.mutex:
                self._lines.append(line.decode().strip())
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

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.interrupt()

class ProcessLogReader(subprocess.Popen, LogReader):

    def __init__(self, cmd, max_log_lines):
        subprocess.Popen.__init__(self, cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT) 
        LogReader.__init__(self, self.stdout, depth=max_log_lines)
        LogReader.start(self)

    def terminate(self):
        subprocess.Popen.terminate(self)
        LogReader.interrupt(self)

    def __enter__(self):
        subprocess.Popen.__enter__(self)
        return self

    def __exit__(self, *args):
        self.terminate()
        subprocess.Popen.__exit__(self, *args)
        LogReader.__exit__(self, *args)

        self.wait()

if __name__ == "__main__":
    # TEST
    import time
    import re

    with ProcessLogReader(["ros2", "launch", "scripts/simulator.launch"], 1) as process:
        while process.poll() is None:
            if process.is_dirty:
                print(process.get_logs())
            time.sleep(0.1)