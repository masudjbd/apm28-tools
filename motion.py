"""Motion-triggered capture: record the movement, not a fixed window.

Split out from check_imu.py so the trigger logic can be tested without a board.

A fixed window forces the operator to read a prompt, pick up an aircraft and
rotate it, all inside a few seconds. Miss the start and the tool reports zero
rotation, which reads as a hardware fault when it is really a stopwatch
problem. Instead: arm, wait for motion, record until it stops.

The pre-buffer matters. By the time the rate crosses the trigger threshold the
movement has already been going for a moment, and that leading slice is real
rotation. Dropping it biases every measurement low.
"""

import collections


class MotionRecorder:
    """Feed it (t, gyro_xyz) samples; it decides when to start and stop.

    States: waiting -> recording -> done
    """

    def __init__(self, start_rate=0.15, stop_rate=0.08, quiet_seconds=0.8,
                 prebuffer_seconds=0.5, max_seconds=20.0, timeout_seconds=30.0):
        self.start_rate = start_rate
        self.stop_rate = stop_rate
        self.quiet_seconds = quiet_seconds
        self.prebuffer_seconds = prebuffer_seconds
        self.max_seconds = max_seconds
        self.timeout_seconds = timeout_seconds

        self.state = "waiting"
        self.samples = []
        self._pre = collections.deque()
        self._quiet_since = None
        self._started_at = None
        self._first_t = None

    @staticmethod
    def _magnitude(gyro):
        return max(abs(gyro[0]), abs(gyro[1]), abs(gyro[2]))

    def feed(self, t, gyro):
        """Returns the current state after consuming one sample."""
        if self._first_t is None:
            self._first_t = t

        if self.state == "waiting":
            self._pre.append((t, gyro))
            while self._pre and t - self._pre[0][0] > self.prebuffer_seconds:
                self._pre.popleft()

            if self._magnitude(gyro) > self.start_rate:
                self.state = "recording"
                self._started_at = t
                self.samples = list(self._pre)      # keep the leading slice
                self._quiet_since = None
            elif t - self._first_t > self.timeout_seconds:
                self.state = "timeout"
            return self.state

        if self.state == "recording":
            self.samples.append((t, gyro))
            if self._magnitude(gyro) < self.stop_rate:
                if self._quiet_since is None:
                    self._quiet_since = t
                elif t - self._quiet_since >= self.quiet_seconds:
                    self.state = "done"
            else:
                self._quiet_since = None

            if t - self._started_at > self.max_seconds:
                self.state = "done"
        return self.state

    def duration(self):
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1][0] - self.samples[0][0]
