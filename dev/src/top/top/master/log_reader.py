from collections import deque
from threading import Thread, Lock, Event
import select
import subprocess
import signal

class LogReader(Thread):

    def __init__(self, stream, depth, transform=None, dup=None):
        assert depth > 0
        super().__init__()
        self._stream = stream
        self._transform = transform
        self._lines = deque(maxlen=depth)
        self._dirty = False
        self.mutex = Lock()
        self.interrupted = Event()
        self.dup = dup

    def run(self):
        dup = open(self.dup, "w") if self.dup is not None else None
        try:
            while not self.interrupted.is_set():
                try:
                    readable, _, _ = select.select([self._stream], [], [], 1)
                    if self.interrupted.is_set():
                        break
                    if readable == []:
                        continue

                    line = self._stream.readline()
                    if not line or self.interrupted.is_set():
                        break
                    line = line.decode().strip()
                    if dup is not None:
                        print(line, file=dup)

                except:
                    # Stream closed (process ended)
                    break

                if self._transform is not None:
                    line = self._transform(line)
                    if not line: continue

                if isinstance(line, str): line = (line,)

                with self.mutex:
                    for line_ in line:
                        self._lines.append(line_)
                    self._dirty = True
        finally:
            if dup is not None:
                dup.close()

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

    def __exit__(self, exc_type, exc_value, traceback):
        self.interrupt()

class ProcessLogReader(subprocess.Popen, LogReader):

    def __init__(self, cmd, max_log_lines, transform=None, dup=None):
        subprocess.Popen.__init__(self, cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT) 
        LogReader.__init__(self, self.stdout, depth=max_log_lines, transform=transform, dup=dup)
        LogReader.start(self)

    def terminate(self):
        self.send_signal(signal.SIGINT)

    def __enter__(self):
        subprocess.Popen.__enter__(self)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate()
        subprocess.Popen.__exit__(self, exc_type, exc_value, traceback)
        self.wait()

        LogReader.__exit__(self, exc_type, exc_value, traceback)

if __name__ == "__main__":
    # TEST
    import time
    import re

    LOG_MATCH_PATTERN = r'\[\w+\]: [\w\W]+'
    
    with ProcessLogReader(["ros2", "launch", "scripts/simulator.launch"], 1, lambda line: [*re.findall(LOG_MATCH_PATTERN, line),""][0]) as process:
        while process.poll() is None:
            if process.is_dirty:
                print(process.get_logs())
            time.sleep(0.1)