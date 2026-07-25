from __future__ import annotations

import logging
import threading
from typing import Any

from .actions import Action

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - deployment diagnostic
    serial = None
    list_ports = None


LOGGER = logging.getLogger(__name__)
EMERGENCY_STOP_COMMAND = "$DST!"
PUMP_OFF_COMMAND = "#005P1500T0500!"


def available_serial_ports(default: str = "COM6") -> list[str]:
    ports = [] if list_ports is None else [item.device for item in list_ports.comports()]
    if default not in ports:
        ports.append(default)
    return sorted(set(ports), key=str.casefold)


class SerialArmController:
    """Single owner of the arm serial port; never connects or moves on construction."""

    def __init__(
        self,
        *,
        baudrate: int = 115200,
        write_timeout_seconds: float = 1.0,
        dry_run: bool = False,
    ) -> None:
        if baudrate != 115200:
            raise ValueError("The deployed V0.1 protocol requires 115200 baud")
        self.baudrate = baudrate
        self.write_timeout_seconds = float(write_timeout_seconds)
        self.dry_run = bool(dry_run)
        self.port: str | None = None
        self._connection: Any | None = None
        self._simulated_connected = False
        self._write_lock = threading.RLock()
        self.dry_run_commands: list[tuple[str, str]] = []

    @property
    def is_connected(self) -> bool:
        if self.dry_run:
            return self._simulated_connected
        return bool(self._connection is not None and self._connection.is_open)

    def connect(self, port: str) -> None:
        selected = port.strip()
        if not selected:
            raise ValueError("Serial port cannot be empty")
        with self._write_lock:
            if self.is_connected:
                if self.port == selected:
                    return
                raise RuntimeError(f"Serial port already connected: {self.port}")
            if self.dry_run:
                self.port = selected
                self._simulated_connected = True
                LOGGER.info("SERIAL DRY-RUN CONNECTED %s %d", selected, self.baudrate)
                return
            if serial is None:
                raise RuntimeError("pyserial is not installed; run scripts/install_dependencies.bat")
            try:
                self._connection = serial.Serial(
                    port=selected,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    write_timeout=self.write_timeout_seconds,
                )
            except Exception as exc:
                self._connection = None
                raise RuntimeError(f"Cannot open {selected} at {self.baudrate} baud: {exc}") from exc
            self.port = selected
            LOGGER.info("SERIAL CONNECTED %s %d", selected, self.baudrate)

    def disconnect(self) -> None:
        with self._write_lock:
            connection = self._connection
            self._connection = None
            self._simulated_connected = False
            old_port = self.port
            self.port = None
            if connection is not None:
                try:
                    connection.close()
                finally:
                    LOGGER.info("SERIAL DISCONNECTED %s", old_port or "-")
            elif old_port:
                LOGGER.info("SERIAL DRY-RUN DISCONNECTED %s", old_port)

    def send_action(self, action: Action) -> None:
        self.write(action.command, label=action.name)

    def emergency_stop(self) -> None:
        self.write(EMERGENCY_STOP_COMMAND, label="ESTOP")

    def pump_off(self) -> None:
        self.write(PUMP_OFF_COMMAND, label="PUMP_OFF")

    def beep(self, times: int = 1, duration_ms: int = 100) -> list[str]:
        """Best-effort arm buzzer via multiple protocol candidates.

        V0.1 PWM firmware may ignore unknown commands; we try several known forms.
        Returns the raw command strings that were written.
        """
        count = max(1, min(5, int(times)))
        duration = max(20, min(1000, int(duration_ms)))
        candidates = [
            f"$BEEP:{count},{duration}!",
            f"beep,{count}" + "\r\n",
            f"beep,{count}" + "\n",
        ]
        sent: list[str] = []
        for cmd in candidates:
            self.write(cmd, label="BEEP")
            sent.append(cmd)
        return sent

    def write(self, command: str, *, label: str = "RAW") -> None:
        if not command or not command.isascii():
            raise ValueError("Serial command must be non-empty ASCII")
        with self._write_lock:
            if not self.is_connected:
                raise RuntimeError("Serial port is not connected")
            LOGGER.info("TX %s %s", label, command)
            if self.dry_run:
                self.dry_run_commands.append((label, command))
                return
            try:
                assert self._connection is not None
                self._connection.write(command.encode("ascii"))
                self._connection.flush()
            except Exception as exc:
                LOGGER.exception("SERIAL ERROR while sending %s", label)
                raise RuntimeError(f"Serial write failed: {exc}") from exc

    def close(self) -> None:
        self.disconnect()

    def __enter__(self) -> "SerialArmController":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
