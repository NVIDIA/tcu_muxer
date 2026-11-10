#!/usr/bin/env python3
"""Reopen a connected physical PTY after a silent-stall decision, retaining
the process and client PTY, and quarantine a partial tag across the data gap.
Only the watchdog decision is injected by the linker fixture.
"""

from __future__ import annotations

import os
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
import tty
from pathlib import Path

from test_physical_uart_hup import (
    RESET,
    read_exact,
    read_named_pty,
    read_until,
)


def main() -> None:
    binary = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="tcu-silent-stall-") as directory:
        base = Path(directory)
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        physical = base / "physical"
        physical.symlink_to(os.ttyname(slave))
        trigger = base / "trigger"
        env = dict(os.environ, TCU_TEST_STALL_TRIGGER=str(trigger))
        client = -1
        with (base / "stderr").open("wb") as stderr:
            process = subprocess.Popen(
                [binary, "-d", str(physical), "-p", "1000"],
                stdout=subprocess.PIPE,
                stderr=stderr,
                env=env,
                bufsize=0,
            )
            try:
                client_path = read_named_pty(process, "CCPLEX")
                assert RESET in read_until(master, RESET, 5)
                client = os.open(
                    client_path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY
                )
                tty.setraw(client)
                os.write(master, b"\xff\xe9BEFORE\n")
                assert read_exact(client, 7, 5) == b"BEFORE\n"
                # Stop halfway through a control sequence before declaring
                # the RX transport stalled. No endpoint or PTY is closed.
                os.write(master, b"\xff")
                time.sleep(0.15)
                trigger.touch()
                assert RESET in read_until(master, RESET, 5)
                assert process.poll() is None
                assert Path(client_path).exists()
                # Old escape state must not interpret this first E9 as a
                # selector; everything before the next FF E9 is quarantined.
                os.write(master, b"\xe9UNTRUSTED\n\xff\xe9AFTER\n")
                assert read_exact(client, 6, 5) == b"AFTER\n"
                assert not select.select([client], [], [], 0.15)[0]
                os.write(client, b"TX_STILL_WORKS\n")
                assert b"TX_STILL_WORKS\n" in read_until(
                    master, b"TX_STILL_WORKS\n", 5
                )
                process.send_signal(signal.SIGUSR1)
                time.sleep(0.2)
                process.send_signal(signal.SIGTERM)
                assert process.wait(timeout=5) == 0
                log = (base / "stderr").read_text()
                assert "generation=2" in log, log
                assert "attempts=1 successes=1 failures=0" in log, log
                assert "quarantined_bytes=11 resynchronizations=1" in log, log
                print(
                    "PASS silent stall without HUP: same process and client PTY; "
                    "RX/TX resume and partial-tag gap is quarantined"
                )
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
                if client >= 0:
                    os.close(client)
                os.close(master)
                os.close(slave)


if __name__ == "__main__":
    main()
