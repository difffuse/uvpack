"""Bounded, Blender-independent transport for the cancel_file_v1 CLI.

Reader threads keep both output pipes moving while a writer supplies the request.
The UI thread only consumes a coalesced incumbent and a checked final response.
Call close() in a finally block, including after a successful pump(). Do not wait
or poll the exposed Popen directly: the monitor owns reaping and tree cleanup.
"""
import json
import errno
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time


def parse_stream_lines(buffer):
    """Legacy tolerant line splitter. Live sessions use the strict parser below."""
    events = []
    while '\n' in buffer:
        line, buffer = buffer.split('\n', 1)
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if isinstance(event, dict):
            events.append(event)
    return events, buffer


def _wait_unreaped(pid, nohang=False):
    """Observe exit without releasing the PID, including older macOS Pythons."""
    if hasattr(os, 'waitid'):
        return os.waitid(os.P_PID, pid, os.WEXITED | os.WNOWAIT |
                         (os.WNOHANG if nohang else 0)) is not None
    if sys.platform != 'darwin':
        raise RuntimeError('safe process cleanup requires waitid/WNOWAIT')
    # Some macOS Python builds omit os.waitid although Darwin provides waitid.
    # Layout and constants are Darwin's sys/signal.h and sys/wait.h ABI, not the
    # different Linux siginfo_t layout. c_void_p has sigval union size/alignment.
    import ctypes
    class SigInfo(ctypes.Structure):
        _fields_ = [('signo', ctypes.c_int), ('error', ctypes.c_int), ('code', ctypes.c_int),
                    ('pid', ctypes.c_int32), ('uid', ctypes.c_uint32), ('status', ctypes.c_int),
                    ('address', ctypes.c_void_p), ('value', ctypes.c_void_p),
                    ('band', ctypes.c_long), ('padding', ctypes.c_ulong * 7)]
    waitid = ctypes.CDLL(None, use_errno=True).waitid
    waitid.argtypes = [ctypes.c_uint, ctypes.c_uint32, ctypes.POINTER(SigInfo), ctypes.c_int]
    waitid.restype = ctypes.c_int
    info = SigInfo()
    while waitid(1, pid, ctypes.byref(info), 0x04 | 0x20 | (0x01 if nohang else 0)) < 0:
        code = ctypes.get_errno()
        if code != errno.EINTR:
            raise OSError(code, os.strerror(code))
    return info.pid != 0


class _ProcessTree:
    """Own the child's ordinary descendants until the direct child is reaped.

    POSIX uses a new session and waitid(WNOWAIT), reserving the process-group ID
    until cleanup. Windows assigns a kill-on-close job before sending any input.
    UVPack reads that input before doing work; this is not a sandbox for arbitrary
    executables that spawn descendants before reading their request.
    """
    def __init__(self, proc):
        self.proc = proc
        self.lock = threading.Lock()
        self.closed = False
        self.job = None
        if os.name == 'nt':
            self.job = _WindowsJob(proc.pid)

    def kill(self):
        with self.lock:
            if self.closed:
                return
            if self.job is not None:
                self.job.close()
            else:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    # Darwin can reject signals during exit before waitid makes
                    # the owned child waitable. Leave cleanup to the monitor in
                    # that transition; it will retry after observing exit, while
                    # the leader still reserves the group ID. Do not block the UI
                    # or turn the original protocol error into PermissionError.
                    if sys.platform != 'darwin':
                        raise
                    if not _wait_unreaped(self.proc.pid, nohang=True):
                        return
            self.closed = True

    def wait(self):
        if os.name != 'nt':
            # No other session method reaps this PID before kill().
            _wait_unreaped(self.proc.pid)
            self.kill()
            return self.proc.wait()
        code = self.proc.wait()
        self.kill()
        return code


class _WindowsJob:
    def __init__(self, pid):
        import ctypes
        from ctypes import wintypes as w

        class BasicLimits(ctypes.Structure):
            _fields_ = [('process_time', ctypes.c_int64), ('job_time', ctypes.c_int64),
                        ('flags', w.DWORD), ('min_working_set', ctypes.c_size_t),
                        ('max_working_set', ctypes.c_size_t), ('active_processes', w.DWORD),
                        ('affinity', ctypes.c_size_t), ('priority', w.DWORD),
                        ('scheduling', w.DWORD)]

        class IoCounters(ctypes.Structure):
            _fields_ = [(name, ctypes.c_uint64) for name in
                        ('read_ops', 'write_ops', 'other_ops', 'read_bytes', 'write_bytes', 'other_bytes')]

        class ExtendedLimits(ctypes.Structure):
            _fields_ = [('basic', BasicLimits), ('io', IoCounters),
                        ('process_memory', ctypes.c_size_t), ('job_memory', ctypes.c_size_t),
                        ('peak_process_memory', ctypes.c_size_t), ('peak_job_memory', ctypes.c_size_t)]

        api = ctypes.WinDLL('kernel32', use_last_error=True)
        api.CreateJobObjectW.argtypes = [ctypes.c_void_p, w.LPCWSTR]
        api.CreateJobObjectW.restype = w.HANDLE
        api.SetInformationJobObject.argtypes = [w.HANDLE, ctypes.c_int, ctypes.c_void_p, w.DWORD]
        api.SetInformationJobObject.restype = w.BOOL
        api.OpenProcess.argtypes = [w.DWORD, w.BOOL, w.DWORD]
        api.OpenProcess.restype = w.HANDLE
        api.AssignProcessToJobObject.argtypes = [w.HANDLE, w.HANDLE]
        api.AssignProcessToJobObject.restype = w.BOOL
        api.CloseHandle.argtypes = [w.HANDLE]
        api.CloseHandle.restype = w.BOOL
        self.api = api
        self.handle = api.CreateJobObjectW(None, None)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        process = None
        try:
            limits = ExtendedLimits()
            limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if not api.SetInformationJobObject(self.handle, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
                raise ctypes.WinError(ctypes.get_last_error())
            process = api.OpenProcess(0x0101, False, pid)  # SET_QUOTA | TERMINATE
            if not process or not api.AssignProcessToJobObject(self.handle, process):
                raise ctypes.WinError(ctypes.get_last_error())
        except BaseException:
            self.close()
            raise
        finally:
            if process:
                api.CloseHandle(process)

    def close(self):
        if self.handle:
            self.api.CloseHandle(self.handle)
            self.handle = None


def _reject_constant(value):
    raise ValueError('non-finite JSON number: ' + value)


def _finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('non-finite JSON number')
    return number


def _unique_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('duplicate JSON object key: ' + key)
        value[key] = item
    return value


class StreamSession:
    READ_SIZE = 1 << 16
    STDERR_LIMIT = 1 << 16
    RECORD_LIMIT = 64 << 20
    INPUT_LIMIT = 64 << 20
    STOP_GRACE = 10.0

    @classmethod
    def start(cls, cli, request, timeout=600, *, max_record_bytes=None, stream=True):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        limit = cls.RECORD_LIMIT if max_record_bytes is None else max_record_bytes
        if not isinstance(limit, int) or limit <= 0:
            raise ValueError('record limit must be a positive integer')
        options = dict(request.get('options') or {})
        options['stream'] = stream
        payload = json.dumps(dict(request, options=options), allow_nan=False).encode('utf-8')
        if len(payload) > cls.INPUT_LIMIT:
            raise ValueError('request exceeds input byte limit')
        if os.name != 'nt' and sys.platform != 'darwin' and not hasattr(os, 'waitid'):
            raise RuntimeError('safe process cleanup requires waitid/WNOWAIT')
        session = cls()
        session._stream = stream
        session._record_limit = limit
        session._deadline = time.monotonic() + timeout
        session._temp = tempfile.TemporaryDirectory(prefix='uvpack-')
        session.cancel_file = str(Path(session._temp.name) / 'request.cancel')
        command = [os.fspath(cli)] if isinstance(cli, (str, os.PathLike)) else list(cli)
        try:
            session.proc = subprocess.Popen(
                command + ['-', '--cancel-file', session.cancel_file],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                bufsize=0, start_new_session=(os.name != 'nt'))
            session._tree = _ProcessTree(session.proc)
            for name, target, args in (
                    ('stdout', session._read_stdout, ()),
                    ('stderr', session._read_stderr, ()),
                    ('stdin', session._write, (payload,)),
                    ('process', session._wait, ())):
                thread = threading.Thread(target=target, args=args, name='uvpack-' + name)
                thread.start()
                session.threads.append(thread)
            return session
        except BaseException:
            session.close()
            raise

    def __init__(self):
        self.proc = None
        self._tree = None
        self._temp = None
        self.threads = []
        self._lock = threading.Lock()
        self.exited = threading.Event()
        self._stdout_eof = threading.Event()
        self._stderr_eof = threading.Event()
        self._stderr = bytearray()
        self._pending = None
        self._done = None
        self._received = 0
        self.improved = 0
        self.layout = None
        self.stopping = 0.0
        self.error = ''
        self._status = 'running'
        self._closed = False

    @property
    def stderr_tail(self):
        with self._lock:
            return self._stderr.decode('utf-8', 'replace')

    def _fail(self, message):
        with self._lock:
            if not self.error:
                self.error = message

    def _write(self, payload):
        try:
            data = memoryview(payload)
            while data:
                written = self.proc.stdin.write(data)
                if not written:
                    raise OSError('stdin write made no progress')
                data = data[written:]
        except BrokenPipeError:
            # An early structured rejection is allowed to leave input unread.
            pass
        except OSError as exc:
            # Windows reports a closed read end as EINVAL on some runtimes.
            if os.name != 'nt' or exc.errno != 22:
                self._fail('stdin: ' + str(exc))
        finally:
            self.proc.stdin.close()

    def _record(self, raw):
        if len(raw) > self._record_limit:
            raise ValueError('stdout record exceeds byte limit')
        value = json.loads(raw.decode('utf-8'), parse_constant=_reject_constant,
                           parse_float=_finite_number, object_pairs_hook=_unique_object)
        if not isinstance(value, dict) or value.get('dialect') != 'uvpack':
            raise ValueError('response is not a uvpack object')
        kind = value.get('event') if self._stream else 'done'
        # The CLI can reject arguments/input before recognizing streaming mode.
        if kind is None and value.get('valid') is False and isinstance(value.get('error'), str):
            kind = 'done'
        if kind not in ('improved', 'done'):
            raise ValueError('unknown stream event')
        if kind == 'done' and type(value.get('valid')) is not bool:
            raise ValueError('final result needs boolean valid')
        with self._lock:
            if self._done is not None:
                raise ValueError('data after final result')
            if kind == 'done':
                self._done = value
            else:
                self._pending = value
                self._received += 1

    def _read_stdout(self):
        pending = bytearray()
        try:
            while True:
                chunk = self.proc.stdout.read(self.READ_SIZE)
                if not chunk:
                    break
                pending.extend(chunk)
                if self._stream:
                    while True:
                        end = pending.find(b'\n')
                        if end < 0:
                            break
                        line = bytes(pending[:end])
                        del pending[:end + 1]
                        if line.strip():
                            self._record(line)
                if len(pending) > self._record_limit:
                    raise ValueError('stdout record exceeds byte limit')
            if pending.strip():
                self._record(bytes(pending))
        except (OSError, ValueError, RecursionError) as exc:
            self._fail('stdout protocol: ' + str(exc))
        finally:
            self.proc.stdout.close()
            self._stdout_eof.set()

    def _read_stderr(self):
        try:
            while True:
                chunk = self.proc.stderr.read(self.READ_SIZE)
                if not chunk:
                    break
                with self._lock:
                    self._stderr.extend(chunk)
                    del self._stderr[:-self.STDERR_LIMIT]
        except OSError as exc:
            self._fail('stderr: ' + str(exc))
        finally:
            self.proc.stderr.close()
            self._stderr_eof.set()

    def _wait(self):
        try:
            self._tree.wait()
        except OSError as exc:
            self._fail('process cleanup: ' + str(exc))
        finally:
            self.exited.set()

    def drain(self):
        """Take the latest pending incumbent; the terminal record stays private."""
        with self._lock:
            value, self._pending = self._pending, None
            self.improved = self._received
        return [] if value is None else [value]

    def interrupt(self):
        if self.stopping or self._closed or self._status != 'running':
            return
        self.stopping = time.monotonic()
        try:
            Path(self.cancel_file).touch()
        except OSError as exc:
            self._fail('cannot signal cancellation: ' + str(exc))

    def pump(self, on_improved):
        if self._status != 'running':
            return self._status
        complete = self.exited.is_set() and self._stdout_eof.is_set() and self._stderr_eof.is_set()
        if not self.error and complete:
            if self._done is None:
                self._fail(f'exit {self.proc.returncode}: no final result; {self.stderr_tail.strip()}')
            elif self.proc.returncode not in (0, 1, 2):
                self._fail(f'unexpected exit {self.proc.returncode}; {self.stderr_tail.strip()}')
            elif self._done['valid'] != (self.proc.returncode == 0):
                self._fail('result validity contradicts process exit')
        now = time.monotonic()
        if not complete:
            if now >= self._deadline:
                self._fail('uvpack_cli timeout')
            elif self.stopping and now - self.stopping > self.STOP_GRACE:
                self._fail('cancellation was not answered')
        if self.error:
            self._status = 'failed'
            self.layout = None
            self._tree.kill()
            return self._status
        for value in self.drain():
            on_improved(value)
        if complete:
            self.layout = self._done
            self._status = 'done'
        return self._status

    def close(self):
        """Kill/reap before joining I/O threads; idempotent on every exit path."""
        if self._closed:
            return
        self._closed = True
        try:
            if self.proc is not None:
                if self._tree is not None:
                    self._tree.kill()
                else:
                    self.proc.kill()
                for thread in self.threads:
                    thread.join()
                self.proc.wait()
                for pipe in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
                    pipe.close()
        finally:
            if self._temp is not None:
                self._temp.cleanup()


def run_cli(cli, request, timeout=600):
    """Return (structured result, error), preserving documented result exits."""
    session = None
    try:
        session = StreamSession.start(cli, request, timeout, stream=False)
        while True:
            status = session.pump(lambda value: None)
            if status != 'running':
                return (session.layout, '') if status == 'done' else (None, session.error)
            time.sleep(.01)
    except (OSError, ValueError, RuntimeError) as exc:
        return None, f'could not run uvpack_cli: {exc}'
    finally:
        if session is not None:
            session.close()
