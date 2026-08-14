#!/usr/bin/env python3
"""Exercise physical-UART replacement while the TX worker is backpressured.

Two pseudoterminals model the old and replacement physical UART endpoints. A
stable symlink models /dev/ttyUSB0: the test repoints it and closes the old
master while CCPLEX data is queued. The muxer must reopen the replacement,
write its complete reset sequence before any queued bytes, remain alive, and
deliver a sentinel submitted through the same CCPLEX PTY.
"""

from __future__ import annotations

import os
import pty
import re
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time
import tty
from pathlib import Path

RESET = b"\x00\xff\xfd"
SENTINEL = b"TCU_HUP_REOPEN_SENTINEL_0123456789\n"
CCPLEX_NAME = "CCPLEX"
CCPLEX_SELECTOR = b"\xff\xe9"
LIFECYCLE_RE = re.compile(
    rb"physical TTY lifecycle: generation=(\d+).*requests=(\d+) "
    rb"attempts=(\d+) successes=(\d+) failures=(\d+) tx_retries=(\d+)"
)


def read_until(fd: int, marker: bytes, timeout: float) -> bytes:
    """Read nonblocking PTY data until marker is present or timeout expires."""
    deadline = time.monotonic() + timeout
    output = bytearray()
    while marker not in output and time.monotonic() < deadline:
        ready, _, _ = select.select(
            [fd], [], [], min(0.1, deadline - time.monotonic())
        )
        if not ready:
            continue
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            continue
        except OSError as error:
            # EIO is how a PTY master reports that its slave has not opened yet.
            if error.errno == 5:
                time.sleep(0.01)
                continue
            raise
        if chunk:
            output.extend(chunk)
    if marker not in output:
        raise AssertionError(
            f"timed out waiting for {marker!r}; captured {len(output)} bytes"
        )
    return bytes(output)


def read_named_pty(process: subprocess.Popen[bytes], name: str) -> str:
    """Resolve one generated VSER PTY from the muxer's startup inventory."""
    assert process.stdout is not None
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        ready, _, _ = select.select([process.stdout], [], [], 0.1)
        if not ready:
            if process.poll() is not None:
                break
            continue
        line = process.stdout.readline()
        if not line:
            break
        fields = line.decode("utf-8", errors="strict").rstrip("\n").split("\t")
        if len(fields) == 2 and fields[1] == name:
            return fields[0]
    raise AssertionError(f"muxer did not publish the {name} PTY")


def write_all(fd: int, payload: bytes, result: list[BaseException]) -> None:
    """Submit one logical CCPLEX transaction, retaining any writer failure."""
    offset = 0
    try:
        while offset < len(payload):
            offset += os.write(fd, payload[offset:])
    except BaseException as error:  # Propagate thread failures to the test.
        result.append(error)


def write_exact(fd: int, payload: bytes, timeout: float = 10) -> None:
    """Write a complete nonblocking physical-UART fixture stream."""
    deadline = time.monotonic() + timeout
    offset = 0
    while offset < len(payload) and time.monotonic() < deadline:
        _, writable, _ = select.select(
            [], [fd], [], min(0.1, deadline - time.monotonic())
        )
        if not writable:
            continue
        try:
            offset += os.write(fd, payload[offset:])
        except BlockingIOError:
            continue
    if offset != len(payload):
        raise AssertionError(
            f"wrote {offset}/{len(payload)} physical RX fixture bytes"
        )


def read_exact(fd: int, length: int, timeout: float) -> bytes:
    """Read an exact PTY byte count without depending on write boundaries."""
    deadline = time.monotonic() + timeout
    output = bytearray()
    while len(output) < length and time.monotonic() < deadline:
        ready, _, _ = select.select(
            [fd], [], [], min(0.1, deadline - time.monotonic())
        )
        if not ready:
            continue
        output.extend(os.read(fd, length - len(output)))
    if len(output) != length:
        raise AssertionError(
            f"received {len(output)}/{length} physical RX bytes"
        )
    return bytes(output)


def replace_symlink(link: Path, target: str) -> None:
    """Atomically change the device pathname seen by the muxer's reopen."""
    replacement = link.with_suffix(".new")
    replacement.symlink_to(target)
    os.replace(replacement, link)


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit(f"usage: {sys.argv[0]} /path/to/tcu_muxer")

    muxer = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="tcu-hup-") as temporary_dir:
        device = Path(temporary_dir) / "physical-uart"
        old_master, old_slave = pty.openpty()
        new_master, new_slave = pty.openpty()
        tty.setraw(old_slave)
        tty.setraw(new_slave)
        os.set_blocking(old_master, False)
        os.set_blocking(new_master, False)
        device.symlink_to(os.ttyname(old_slave))

        process = subprocess.Popen(
            [muxer, "-d", str(device), "-p", "1000"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            # Keep startup inventory reads aligned with select(2). Buffered
            # pipes could prefetch later PTY lines into Python while leaving
            # the underlying descriptor no longer readable.
            bufsize=0,
        )
        ccplex_fd = -1
        writer: threading.Thread | None = None
        writer_errors: list[BaseException] = []
        try:
            ccplex_path = read_named_pty(process, CCPLEX_NAME)
            # Initial device setup must complete before the fault is injected.
            read_until(old_master, RESET, 5)
            ccplex_fd = os.open(ccplex_path, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(ccplex_fd)

            # Leave the old physical master unread after its reset. This fills
            # the physical PTY and guarantees TX is active/backpressured when
            # the endpoint disappears, rather than testing an idle reopen.
            payload = b"Q" * (512 * 1024) + SENTINEL
            writer = threading.Thread(
                target=write_all,
                args=(ccplex_fd, payload, writer_errors),
                daemon=True,
            )
            writer.start()
            time.sleep(0.1)

            # Publish the replacement path before closing the old endpoint so
            # the first serialized reopen attempt has a valid device to open.
            replace_symlink(device, os.ttyname(new_slave))
            os.close(old_master)
            old_master = -1

            replacement_stream = read_until(new_master, SENTINEL, 20)
            if not replacement_stream.startswith(RESET):
                raise AssertionError(
                    "replacement stream did not begin with one complete TCU reset: "
                    f"{replacement_stream[:16].hex()}"
                )
            if process.poll() is not None:
                raise AssertionError(
                    "muxer exited during concurrent TX and HUP"
                )

            writer.join(timeout=10)
            if writer.is_alive():
                raise AssertionError(
                    "CCPLEX producer remained blocked after reopen"
                )
            if writer_errors:
                raise writer_errors[0]

            # Exercise the opposite direction after replacement as well. The
            # logical payload exceeds six maximum 16-KiB physical reads and
            # includes every byte value. Escaping 0xff models the real UTC
            # wire format; the CCPLEX PTY must receive the original bytes
            # exactly after selector parsing and batched per-VSER routing.
            rx_payload = bytes(range(256)) * 400
            encoded_payload = rx_payload.replace(b"\xff", b"\xff\xff")
            write_exact(new_master, CCPLEX_SELECTOR + encoded_payload)
            observed_payload = read_exact(ccplex_fd, len(rx_payload), 10)
            if observed_payload != rx_payload:
                raise AssertionError(
                    "physical RX batching changed CCPLEX bytes"
                )

            # SIGUSR1 gives the regression a mid-run snapshot; graceful
            # shutdown prints the final snapshot used for assertions below.
            process.send_signal(signal.SIGUSR1)
            time.sleep(0.1)
            process.send_signal(signal.SIGTERM)
            _, stderr = process.communicate(timeout=10)
            if process.returncode != 0:
                raise AssertionError(
                    f"muxer exited with {process.returncode}:\n"
                    f"{stderr.decode(errors='replace')}"
                )

            matches = LIFECYCLE_RE.findall(stderr)
            if not matches:
                raise AssertionError(
                    "missing descriptor-lifecycle statistics:\n"
                    + stderr.decode(errors="replace")
                )
            generation, requests, attempts, successes, failures, tx_retries = (
                map(int, matches[-1])
            )
            if generation < 2 or requests < 1 or attempts < 1 or successes < 1:
                raise AssertionError(f"reopen was not proven: {matches[-1]!r}")
            if failures != 0:
                raise AssertionError(
                    f"unexpected reopen failure: {matches[-1]!r}"
                )
            if tx_retries < 1:
                raise AssertionError(
                    "test did not make TX observe the concurrent disconnect: "
                    f"{matches[-1]!r}"
                )
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
            if writer is not None:
                writer.join(timeout=1)
            if ccplex_fd >= 0:
                os.close(ccplex_fd)
            for fd in (old_master, old_slave, new_master, new_slave):
                if fd >= 0:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
