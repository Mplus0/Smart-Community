"""Subprocess manager for ``roslaunch`` and shell helpers.

Behaviour contract (see ``WORKBENCH.md`` §5):

* Each child is launched in its own process group via ``preexec_fn=
  os.setsid`` so we can kill the whole tree with ``killpg``.
* Stop escalation ``SIGINT → SIGTERM → SIGKILL`` with 10 s and 5 s
  grace periods.
* A launch that is already running refuses re-start; the current ``pid``
  and ``args`` are returned to the caller.
* stdout + stderr are tee'd to a per-run log file under
  ``~/.ros/community/infra/logs/`` and every line is also published
  on the :class:`~infra.state_bus.StateBus` so the UI can tail live.
* On interpreter shutdown an ``atexit`` hook sends ``SIGINT`` to every
  surviving group so Ctrl+C on the infra never leaks a ``roslaunch``.

The manager is intentionally isolated from FastAPI/ROS; the only external
dependency is :mod:`infra.state_bus`.
"""

from __future__ import annotations

import atexit
import logging
import os
import shlex
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from . import config
from .state_bus import StateBus, get_bus

_LOG = logging.getLogger(__name__)

_SIGINT_GRACE_S = 10.0
_SIGTERM_GRACE_S = 5.0


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------


@dataclass
class LaunchRecord:
    """Everything the UI needs to describe a launched child."""

    name: str
    pid: int
    pgid: int
    args: Dict[str, str]
    start_ts: float
    log_path: Path
    cmd: List[str]
    status: str = "running"             # running | exited | failed
    exit_code: Optional[int] = None
    stopped_ts: Optional[float] = None
    stop_reason: Optional[str] = None

    def to_dict(self) -> Dict[str, object]:
        base: Dict[str, object] = {
            "name": self.name,
            "pid": self.pid,
            "pgid": self.pgid,
            "args": dict(self.args),
            "status": self.status,
            "start_ts": self.start_ts,
            "log_path": str(self.log_path),
            "cmd": list(self.cmd),
        }
        now = time.time()
        if self.status == "running":
            base["since_s"] = max(0.0, now - self.start_ts)
        else:
            base["exit_code"] = self.exit_code
            base["stopped_ts"] = self.stopped_ts
            base["stop_reason"] = self.stop_reason
            if self.stopped_ts is not None:
                base["since_s"] = max(0.0, now - self.stopped_ts)
        return base


@dataclass
class _ChildProcess:
    """Private pairing of :class:`subprocess.Popen` and its record."""

    popen: subprocess.Popen
    record: LaunchRecord
    log_file: "object"              # TextIO, kept open for the reader thread
    reader_thread: threading.Thread = field(repr=False)


# ---------------------------------------------------------------------------
# LaunchManager
# ---------------------------------------------------------------------------


class LaunchManager:
    """Thread-safe registry of infra-owned child processes."""

    def __init__(self, bus: Optional[StateBus] = None) -> None:
        self._bus = bus or get_bus()
        self._lock = threading.RLock()
        self._children: Dict[str, _ChildProcess] = {}
        self._history: Dict[str, LaunchRecord] = {}
        atexit.register(self._atexit_stop_all)

    # -- queries -----------------------------------------------------------

    def status(self, name: str) -> Optional[LaunchRecord]:
        """Return the live record if running, else the last known record."""
        with self._lock:
            child = self._children.get(name)
            if child is not None:
                return child.record
            return self._history.get(name)

    def all_records(self) -> List[LaunchRecord]:
        """Snapshot of every record the manager has ever seen.

        Running records come first in start order; history follows.
        """
        with self._lock:
            live = [c.record for c in self._children.values()]
            historical = [r for n, r in self._history.items() if n not in self._children]
            return live + historical

    def is_running(self, name: str) -> bool:
        with self._lock:
            return name in self._children

    # -- start / stop ------------------------------------------------------

    def start(
        self,
        name: str,
        cmd: List[str],
        args: Optional[Dict[str, str]] = None,
        env: Optional[Dict[str, str]] = None,
    ) -> LaunchRecord:
        """Spawn ``cmd`` in its own process group.

        Raises :class:`AlreadyRunning` if ``name`` is still live.
        """
        args = dict(args or {})
        with self._lock:
            existing = self._children.get(name)
            if existing is not None:
                raise AlreadyRunning(name, existing.record)

            config.ensure_runtime_dirs()
            log_path = self._log_path(name)
            log_file = log_path.open("a", buffering=1, encoding="utf-8", errors="replace")
            self._write_header(log_file, name, cmd, args)

            popen_kwargs: Dict[str, object] = dict(
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                text=True,
                env={**os.environ, **(env or {})},
            )
            # ``preexec_fn`` is POSIX-only. On Windows (dev workstations)
            # we fall back to a plain spawn; the manager still works for
            # unit tests that do not rely on ``killpg``.
            if os.name == "posix":
                popen_kwargs["preexec_fn"] = os.setsid  # type: ignore[assignment]
            else:  # pragma: no cover — Windows dev convenience only
                popen_kwargs["creationflags"] = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

            popen = subprocess.Popen(cmd, **popen_kwargs)  # type: ignore[arg-type]
            try:
                pgid = os.getpgid(popen.pid) if os.name == "posix" else popen.pid
            except (ProcessLookupError, OSError):
                pgid = popen.pid

            record = LaunchRecord(
                name=name,
                pid=popen.pid,
                pgid=pgid,
                args=args,
                start_ts=time.time(),
                log_path=log_path,
                cmd=list(cmd),
            )

            reader = threading.Thread(
                target=self._pump_output,
                args=(name, popen, log_file),
                daemon=True,
                name=f"infra-log-{name}",
            )
            child = _ChildProcess(popen=popen, record=record, log_file=log_file, reader_thread=reader)
            self._children[name] = child
            reader.start()
            _LOG.info("started launch %s pid=%s cmd=%s", name, popen.pid, " ".join(shlex.quote(c) for c in cmd))
            return record

    def stop(self, name: str, reason: str = "user") -> LaunchRecord:
        """Escalate ``SIGINT → SIGTERM → SIGKILL`` until the child exits."""
        with self._lock:
            child = self._children.get(name)
            if child is None:
                raise NotRunning(name)

        record = child.record
        record.stop_reason = reason
        popen = child.popen
        pgid = record.pgid

        for sig, grace in ((signal.SIGINT, _SIGINT_GRACE_S), (signal.SIGTERM, _SIGTERM_GRACE_S)):
            if popen.poll() is not None:
                break
            self._signal_group(pgid, popen, sig)
            if self._wait(popen, grace) is not None:
                break
        else:
            if popen.poll() is None:
                self._signal_group(pgid, popen, signal.SIGKILL)
                self._wait(popen, 2.0)

        record.exit_code = popen.poll()
        record.stopped_ts = time.time()
        record.status = "exited" if record.exit_code == 0 else "failed"

        with self._lock:
            self._children.pop(name, None)
            self._history[name] = record

        try:
            child.log_file.close()
        except Exception:  # pragma: no cover — defensive, logs already closed
            pass

        _LOG.info("stopped launch %s code=%s reason=%s", name, record.exit_code, reason)
        return record

    def stop_all(self, reason: str = "shutdown") -> None:
        """Best-effort stop of every live child."""
        for name in list(self._children.keys()):
            try:
                self.stop(name, reason=reason)
            except NotRunning:
                continue
            except Exception:  # pragma: no cover — logged, never raised
                _LOG.exception("error stopping %s", name)

    # -- log tailing -------------------------------------------------------

    def tail(self, name: str, lines: int = 400) -> List[str]:
        """Return the last ``lines`` lines from ``name``'s log file."""
        record = self.status(name)
        if record is None:
            raise NotRunning(name)
        return _tail_file(record.log_path, lines)

    def log_path(self, name: str) -> Path:
        record = self.status(name)
        if record is None:
            raise NotRunning(name)
        return record.log_path

    # -- internals ---------------------------------------------------------

    @staticmethod
    def _log_path(name: str) -> Path:
        ts = time.strftime("%Y%m%d_%H%M%S")
        return config.LOG_DIR / f"{name}_{ts}.log"

    @staticmethod
    def _write_header(log_file, name: str, cmd: List[str], args: Dict[str, str]) -> None:
        header = (
            f"# infra launch {name}\n"
            f"# started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"# args: {args}\n"
            f"# cmd : {' '.join(shlex.quote(c) for c in cmd)}\n"
            f"# ----------------------------------------\n"
        )
        log_file.write(header)

    def _pump_output(self, name: str, popen: subprocess.Popen, log_file) -> None:
        """Forward stdout to the log file and the state bus until EOF."""
        assert popen.stdout is not None
        try:
            for line in popen.stdout:
                try:
                    log_file.write(line)
                except Exception:  # pragma: no cover
                    pass
                self._bus.publish_log(name, line.rstrip("\n"))
        except Exception:  # pragma: no cover — reader threads must not leak
            _LOG.exception("log pump for %s crashed", name)
        finally:
            popen.wait()
            with self._lock:
                child = self._children.get(name)
                if child is not None and child.popen is popen:
                    record = child.record
                    record.exit_code = popen.returncode
                    record.stopped_ts = time.time()
                    record.status = "exited" if popen.returncode == 0 else "failed"
                    record.stop_reason = record.stop_reason or "self-exit"
                    self._children.pop(name, None)
                    self._history[name] = record
            try:
                log_file.close()
            except Exception:  # pragma: no cover
                pass

    @staticmethod
    def _signal_group(pgid: int, popen: subprocess.Popen, sig: int) -> None:
        if os.name == "posix":
            try:
                os.killpg(pgid, sig)
                return
            except ProcessLookupError:
                return
            except PermissionError:  # pragma: no cover — bogus pgid
                _LOG.warning("permission denied signalling pgid=%s", pgid)
                return
        # Windows: best-effort terminate on the direct child only.
        try:  # pragma: no cover — Windows dev only
            if sig == signal.SIGKILL:
                popen.kill()
            else:
                popen.terminate()
        except Exception:
            pass

    @staticmethod
    def _wait(popen: subprocess.Popen, timeout: float) -> Optional[int]:
        try:
            return popen.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def _atexit_stop_all(self) -> None:  # pragma: no cover — interpreter exit
        try:
            self.stop_all(reason="atexit")
        except Exception:
            _LOG.exception("failed to stop launches at atexit")


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------


class LaunchError(RuntimeError):
    """Base class for launch manager errors."""


class AlreadyRunning(LaunchError):
    def __init__(self, name: str, record: LaunchRecord) -> None:
        super().__init__(f"launch {name!r} already running (pid={record.pid})")
        self.name = name
        self.record = record


class NotRunning(LaunchError):
    def __init__(self, name: str) -> None:
        super().__init__(f"launch {name!r} is not running")
        self.name = name


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _tail_file(path: Path, lines: int) -> List[str]:
    """Return the last ``lines`` lines of ``path`` without reading the whole file."""
    if lines <= 0:
        return []
    try:
        size = path.stat().st_size
    except FileNotFoundError:
        return []
    if size == 0:
        return []

    block = 4096
    data = b""
    with path.open("rb") as fh:
        pos = size
        while pos > 0 and data.count(b"\n") <= lines:
            read = min(block, pos)
            pos -= read
            fh.seek(pos)
            data = fh.read(read) + data
    text = data.decode("utf-8", errors="replace")
    return text.splitlines()[-lines:]


# ---------------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------------

_manager: Optional[LaunchManager] = None


def get_manager() -> LaunchManager:
    global _manager
    if _manager is None:
        _manager = LaunchManager()
    return _manager


def reset_manager_for_tests() -> None:
    global _manager
    if _manager is not None:
        _manager.stop_all(reason="reset")
    _manager = None


def build_roslaunch_cmd(package: str, launch_file: str, args: Dict[str, str]) -> List[str]:
    """Return a ``['roslaunch', pkg, file, k:=v, ...]`` command vector.

    Empty-string values are treated as "use the launch default" and are
    dropped so we never pass bare ``key:=`` tokens which ``roslaunch``
    rejects.
    """
    cmd = ["roslaunch", package, launch_file]
    for key, value in (args or {}).items():
        if value is None or value == "":
            continue
        cmd.append(f"{key}:={value}")
    return cmd
