"""Supervised MLX90640 reader and 5 Hz FPGA temperature heartbeat.

The FPGA receives the maximum of the entire 32x24 thermal image. Temperature
acquisition never runs on the inference/task thread. A supervised subprocess
also isolates kernel I2C hangs. No extra Python packages are required.
"""
from __future__ import annotations

import argparse
import ctypes
import errno
import json
import logging
import math
import numbers
import os
from pathlib import Path
import queue
import struct
import subprocess
import sys
import threading
import time

LOG = logging.getLogger("fpga_yolo.thermal")
THERMAL_WIRE = struct.Struct("!4sBBHIhH")
THRESHOLDS = (40, 50, 60)
SAMPLE_MAX_AGE = .8
HEARTBEAT_SECONDS = .2
READER_TIMEOUT = 2.0
RETRY_SECONDS = 2.0
DEFAULT_LIBRARY = Path(__file__).resolve().parent / "mlx90640" / "libthermal_mlx90640.so"


def validate_threshold(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value not in THRESHOLDS:
        raise ValueError("temperature threshold must be 40, 50 or 60 degrees C")
    return int(value)


def encode_thermal(sequence, max_c, *, task_active, threshold_c=40):
    """FTHM v1, network order; signed hundredths of a degree, then threshold."""
    threshold_c = validate_threshold(threshold_c)
    valid = max_c is not None
    if valid and (isinstance(max_c, bool) or not isinstance(max_c, numbers.Real) or
                  not math.isfinite(max_c) or not -40 <= max_c <= 300):
        raise ValueError("invalid MLX90640 maximum temperature")
    flags = (1 if valid else 0) | (2 if task_active else 0)
    return THERMAL_WIRE.pack(b"FTHM", 1, flags, THERMAL_WIRE.size,
                             sequence & 0xffffffff,
                             round(max_c * 100) if valid else 0, threshold_c)


class _NativeProcess:
    """The pipe is read by a daemon thread; poll never waits for I2C or a line."""
    def __init__(self, library_path, destination=None):
        library_path = Path(library_path)
        if not library_path.is_file():
            raise RuntimeError("MLX90640 driver not built; run bash install_thermal.sh")
        self.events = queue.Queue(maxsize=8)
        self.frame = None
        environment = dict(os.environ)
        if destination:
            environment['MLX_FPGA_IP'] = str(destination[0])
            environment['MLX_FPGA_PORT'] = str(destination[1])
        self.process = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--sensor-worker",
             "--library", str(library_path.resolve())],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1, env=environment,
        )
        self.reader = threading.Thread(target=self._read, name="mlx90640-pipe", daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                try:
                    value = json.loads(line)
                    if not isinstance(value, dict):
                        raise ValueError("invalid sensor response")
                    event = (value, time.monotonic())
                except (ValueError, TypeError):
                    event = ({"error": "invalid sensor response"}, time.monotonic())
                try:
                    self.events.put_nowait(event)
                except queue.Full:
                    try:
                        self.events.get_nowait()
                    except queue.Empty:
                        pass
                    self.events.put_nowait(event)
        except (OSError, ValueError):
            pass  # The supervisor closed a terminated process.

    def poll(self):
        latest = None
        while True:
            try:
                event, received_at = self.events.get_nowait()
            except queue.Empty:
                break
            if "error" in event:
                raise RuntimeError(str(event["error"]))
            self.frame = event.get("pixels")
            latest = (event.get("max_c"), received_at)
        if self.process.poll() is not None and latest is None:
            raise RuntimeError("MLX90640 reader stopped")
        return latest

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=.3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                try:
                    self.process.wait(timeout=.3)
                except subprocess.TimeoutExpired:
                    pass
        self.reader.join(timeout=.2)
        # Avoid acquiring a buffered-pipe lock if a stuck kernel process cannot
        # exit even after SIGKILL. The daemon reader cannot block shutdown.
        if not self.reader.is_alive():
            self.process.stdout.close()


class ThermalMonitor:
    def __init__(self, udp_socket, destination, task_active, *, enabled=True,
                 library_path=None, sensor_factory=None, clock=time.monotonic):
        self.socket, self.destination, self.task_active = udp_socket, destination, task_active
        self.enabled, self.clock = bool(enabled), clock
        self.library_path = Path(library_path) if library_path else DEFAULT_LIBRARY
        self.sensor_factory = sensor_factory or (lambda: _NativeProcess(self.library_path, destination))
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.sensor = None
        self.sequence = 0
        self.threshold_c = 40
        self.maximum = None
        self.pixels = None
        self.frame_id = 0
        self.sample_at = None
        self.reader_started = None
        self.next_retry = 0.0
        self.active = False
        self.error = "waiting for thermal sensor" if self.enabled else "disabled"
        self.last_logged_error = None
        self.task_generation = 0
        self.manual_override = None
        self.reader_reset_pending = False

    def reset_task(self):
        """Revoke the old sample/alarm before a new task can gain authority."""
        with self.lock:
            self.task_generation += 1
            self.manual_override = None
            self.reader_reset_pending = True
            self.maximum, self.sample_at, self.pixels = None, None, None
            self.active = False
            self.error = 'waiting for requested inspection at target'
            self._send(None, False, self.threshold_c)

    def set_manual(self, enabled):
        """Explicit web authority to acquire or stop without issuing motion."""
        if type(enabled) is not bool:
            raise ValueError('manual thermal state must be boolean')
        if not self.enabled:
            raise RuntimeError('thermal monitor is disabled')
        with self.lock:
            self.task_generation += 1
            self.manual_override = enabled
            self.reader_reset_pending = True
            self.maximum, self.sample_at, self.pixels = None, None, None
            self.active = False
            self.error = 'waiting for thermal sensor' if enabled else 'thermal stream closed manually'
            self._send(None, False, self.threshold_c)
            return self.snapshot()

    def set_threshold(self, value):
        value = validate_threshold(value)
        with self.lock:
            self.threshold_c = value
        return value

    def _valid(self, now):
        return (self.enabled and self.maximum is not None and self.sample_at is not None
                and 0 <= now - self.sample_at <= SAMPLE_MAX_AGE and not self.error)

    def snapshot(self):
        with self.lock:
            valid = self._valid(self.clock())
            maximum = self.maximum if valid else None
            return dict(enabled=self.enabled, sensor_valid=valid, max_c=maximum,
                        control_mode='auto' if self.manual_override is None else 'manual' if self.manual_override else 'off',
                        threshold_c=self.threshold_c, task_active=self.active,
                        initializing=bool(self.active and self.sensor is not None and self.maximum is None and
                            self.error in ('waiting for thermal sensor', 'waiting for requested inspection at target')),
                        warning=bool(valid and self.active and
                                     round(maximum * 100) >= self.threshold_c * 100),
                        error=self.error if self.error else ("" if valid else "thermal sensor data stale"))

    def start(self):
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._run, name="mlx90640-monitor", daemon=True)
        self.thread.start()

    def frame_snapshot(self):
        """Return only a fresh, complete image from the same alarm acquisition."""
        with self.lock:
            status = self.snapshot()
            available = status['sensor_valid'] and self.pixels is not None
            status.update(frame_available=available, width=32, height=24,
                          frame_id=self.frame_id, pixels=None, min_c=None)
            if available:
                status['pixels'] = [round(value, 2) for value in self.pixels]
                status['min_c'] = min(self.pixels)
                status['age_ms'] = round((self.clock() - self.sample_at) * 1000)
            elif status['sensor_valid']:
                status['error'] = 'Full thermal image unavailable; rebuild the driver with bash install_thermal.sh'
            return status

    def _close_sensor(self):
        sensor, self.sensor = self.sensor, None
        if sensor is not None:
            sensor.close()

    def _fail(self, error, now):
        with self.lock:
            self.maximum, self.sample_at = None, None
            self.pixels = None
            self.error = str(error)
        if self.error != self.last_logged_error:
            LOG.warning("Thermal sensor unavailable: %s", self.error)
            self.last_logged_error = self.error
        self._close_sensor()
        self.next_retry = now + RETRY_SECONDS

    def _send(self, maximum, active, threshold):
        self.sequence = (self.sequence + 1) & 0xffffffff
        try:
            self.socket.sendto(encode_thermal(self.sequence, maximum, task_active=active,
                                              threshold_c=threshold), self.destination)
        except OSError as error:
            LOG.debug("Temperature heartbeat could not be sent: %s", error)

    def _tick(self, now):
        with self.lock:
            generation = self.task_generation
            manual = self.manual_override
            reset_reader = self.reader_reset_pending
            self.reader_reset_pending = False
        if reset_reader:
            self._close_sensor()
            self.reader_started, self.next_retry = None, 0.0
        try:
            active = manual if manual is not None else bool(self.task_active())
        except Exception:
            active = False
            LOG.exception("Could not read thermal task state")
        with self.lock:
            if generation != self.task_generation:
                return
        if self.enabled and active:
            try:
                if self.sensor is None and now >= self.next_retry:
                    self.sensor = self.sensor_factory()
                    self.reader_started = now
                if self.sensor is not None:
                    sample = self.sensor.poll()
                    if sample is not None:
                        now = self.clock()
                        maximum, captured_at = sample
                        # Validate before publishing. Treat malformed native output
                        # as a sensor fault, never as an alarm or a zero reading.
                        encode_thermal(0, maximum, task_active=False)
                        if maximum is None or captured_at > now or now - captured_at > SAMPLE_MAX_AGE:
                            raise RuntimeError("thermal sensor data stale")
                        pixels = getattr(self.sensor, 'frame', None)
                        if pixels is not None:
                            if (not isinstance(pixels, (list, tuple)) or len(pixels) != 768 or
                                any(isinstance(v, bool) or not isinstance(v, numbers.Real) or
                                    not math.isfinite(v) or not -40 <= v <= 300 for v in pixels)):
                                raise RuntimeError('invalid thermal image')
                            if abs(max(pixels) - maximum) > .001:
                                raise RuntimeError('thermal maximum does not match image')
                            pixels = tuple(float(value) for value in pixels)
                        with self.lock:
                            if generation != self.task_generation:
                                return  # A new command revoked this in-flight sample.
                            self.maximum, self.sample_at, self.error = float(maximum), captured_at, ""
                            self.pixels = pixels
                            if pixels is not None:
                                self.frame_id += 1
                        self.last_logged_error = None
                    last_read = self.sample_at if self.sample_at is not None else self.reader_started
                    if now - last_read > READER_TIMEOUT:
                        raise RuntimeError("thermal sensor read timed out")
            except (OSError, RuntimeError, ValueError, TypeError) as error:
                with self.lock:
                    if generation != self.task_generation:
                        return
                    self._fail(error, now)
        else:
            self._close_sensor()
            with self.lock:
                self.maximum, self.sample_at, self.pixels = None, None, None
                self.reader_started, self.next_retry = None, 0.0
                self.error = ('thermal stream closed manually' if manual is False else
                              'waiting for requested inspection at target') if self.enabled else 'disabled'
        with self.lock:
            if generation != self.task_generation:
                return
            self.active = active
            maximum = self.maximum if self._valid(now) else None
            threshold = self.threshold_c
            # Normal calibration/first-frame warmup is not a sensor failure.
            # Real initialization errors still produce an active invalid sample.
            armed = active and (maximum is not None or self.error not in (
                'waiting for thermal sensor', 'waiting for requested inspection at target'))
            self._send(maximum, armed, threshold)

    def _run(self):
        try:
            while not self.stop_event.is_set():
                started = self.clock()
                self._tick(started)
                self.stop_event.wait(max(0, HEARTBEAT_SECONDS - (self.clock() - started)))
        finally:
            self._close_sensor()
            self._send(None, False, self.threshold_c)

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        else:
            self._close_sensor()
            self._send(None, False, self.threshold_c)
        with self.lock:
            self.maximum, self.sample_at, self.active = None, None, False
            self.pixels = None
            self.error = "thermal monitor stopped"


def _sensor_worker(library_path):
    library = None
    try:
        library = ctypes.CDLL(str(Path(library_path).resolve()), use_errno=True)
        library.thermal_open.argtypes = [ctypes.c_float]
        library.thermal_open.restype = ctypes.c_int
        library.thermal_read_max.argtypes = [ctypes.POINTER(ctypes.c_float)]
        library.thermal_read_max.restype = ctypes.c_int
        read_frame = getattr(library, 'thermal_read_frame', None)
        if read_frame is not None:
            read_frame.argtypes = [ctypes.POINTER(ctypes.c_float)]
            read_frame.restype = ctypes.c_int
        library.thermal_close.argtypes = []
        library.thermal_close.restype = None
        result = library.thermal_open(.95)
        if result:
            if ctypes.get_errno() in (errno.EAGAIN, errno.EWOULDBLOCK):
                raise RuntimeError('MLX90640 is already in use; close standalone preview and use the task page thermal preview')
            raise RuntimeError("MLX90640 initialization failed (code %d); check FPGA firmware, Ethernet, address 0x33 and E13/E14 wiring" % result)
        while True:
            if read_frame is not None:
                temperatures = (ctypes.c_float * 768)()
                result = read_frame(temperatures)
                values = list(temperatures)
                event = {'max_c': max(values), 'pixels': values}
            else:
                maximum = ctypes.c_float()
                result = library.thermal_read_max(ctypes.byref(maximum))
                event = {'max_c': maximum.value}
            if result:
                raise RuntimeError("MLX90640 frame read failed (code %d)" % result)
            # The supervisor also validates the value independently.
            print(json.dumps(event, separators=(',', ':')), flush=True)
    except Exception as error:
        print(json.dumps({"error": str(error)}), flush=True)
        return 1
    finally:
        if library is not None and hasattr(library, "thermal_close"):
            library.thermal_close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", default=str(DEFAULT_LIBRARY))
    parser.add_argument("--sensor-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--check", action="store_true", help="read via FPGA Ethernet without motion or alarm commands")
    parser.add_argument("--fpga-ip", default="192.168.10.2")
    parser.add_argument("--fpga-port", type=int, default=5001)
    args = parser.parse_args(argv)
    if args.sensor_worker:
        return _sensor_worker(args.library)
    if not args.check:
        parser.error("use --check for a standalone sensor check")
    # The check uses the same process timeout as the live receiver.
    sensor = None
    try:
        sensor = _NativeProcess(args.library, (args.fpga_ip, args.fpga_port))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            sample = sensor.poll()
            if sample is not None:
                encode_thermal(0, sample[0], task_active=False)
                print("MLX90640 OK: 32x24 full frame, maximum %.2f C" % sample[0])
                return 0
            time.sleep(.05)
        raise RuntimeError("MLX90640 check timed out")
    except (OSError, RuntimeError, ValueError) as error:
        print("MLX90640 unavailable: %s" % error, file=sys.stderr)
        return 1
    finally:
        if sensor is not None:
            sensor.close()


if __name__ == "__main__":
    raise SystemExit(main())
