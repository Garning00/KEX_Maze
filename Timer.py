import time

class Timer:
    def __init__(self, duration_sec):
        self.duration = duration_sec  # How long to wait
        self.start_time = None
        self.running = False

    def start(self):
        if not self.running:
            self.start_time = time.perf_counter()
            self.running = True

    def stop(self):
        self.start_time = None
        self.running = False

    def reset(self):
        self.stop()
        self.start()

    def is_done(self):
        if not self.running or self.start_time is None:
            return False
        return (time.perf_counter() - self.start_time) >= self.duration

    def get_elapsed(self):
        if not self.running or self.start_time is None:
            return 0.0
        return time.perf_counter() - self.start_time