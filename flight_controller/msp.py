from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:  # Unit tests and simulator do not require pyserial.
    serial = None


class MSPError(RuntimeError):
    pass


class MSPTimeout(MSPError):
    pass


class MSPChecksumError(MSPError):
    pass


class MSPCommandError(MSPError):
    pass


@dataclass(frozen=True)
class MSPFrame:
    command: int
    payload: bytes
    error: bool = False


def xor_checksum(data: bytes) -> int:
    result = 0
    for value in data:
        result ^= value
    return result


def build_request(command: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 255:
        raise ValueError("MSPv1 command must be within 0..255")
    if len(payload) > 255:
        raise ValueError("MSPv1 payload must be <= 255 bytes")
    body = bytes((len(payload), command)) + payload
    return b"$M<" + body + bytes((xor_checksum(body),))


def crc8_dvb_s2(data: bytes, initial: int = 0) -> int:
    crc = initial & 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_v2_request(command: int, payload: bytes = b"", flags: int = 0) -> bytes:
    if not 0 <= command <= 0xFFFF:
        raise ValueError("MSPv2 command must be within 0..65535")
    if len(payload) > 0xFFFF:
        raise ValueError("MSPv2 payload must be <= 65535 bytes")
    if not 0 <= flags <= 0xFF:
        raise ValueError("MSPv2 flags must be within 0..255")
    header = struct.pack("<BHH", flags, command, len(payload))
    return b"$X<" + header + payload + bytes((crc8_dvb_s2(header + payload),))


class MSPTransport:
    """Synchronous, retrying MSPv1 transport for one exclusive serial owner."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        *,
        request_timeout: float = 0.8,
        retries: int = 1,
    ) -> None:
        if serial is None:
            raise MSPError("pyserial is missing; run: python -m pip install -r requirements.txt")
        self.port = port
        self.baudrate = baudrate
        self.request_timeout = request_timeout
        self.retries = retries
        self._lock = threading.Lock()
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=1.0,
        )
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "MSPTransport":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _read_exact(self, count: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < count:
            if time.monotonic() >= deadline:
                raise MSPTimeout(f"Timeout reading {count} bytes")
            chunk = self.serial.read(count - len(result))
            if chunk:
                result.extend(chunk)
        return bytes(result)

    def _read_frame(self, deadline: float) -> MSPFrame:
        state = 0
        while time.monotonic() < deadline:
            item = self.serial.read(1)
            if not item:
                continue
            value = item[0]
            if state == 0:
                state = 1 if value == ord("$") else 0
            elif state == 1:
                state = 2 if value == ord("M") else (1 if value == ord("$") else 0)
            else:
                if value not in (ord(">"), ord("!")):
                    state = 1 if value == ord("$") else 0
                    continue
                header = self._read_exact(2, deadline)
                size, command = header
                payload = self._read_exact(size, deadline)
                received = self._read_exact(1, deadline)[0]
                expected = xor_checksum(header + payload)
                if received != expected:
                    raise MSPChecksumError(
                        f"MSP {command} checksum 0x{received:02x}, expected 0x{expected:02x}"
                    )
                return MSPFrame(command, payload, value == ord("!"))
        raise MSPTimeout("No MSP response")

    def _read_v2_frame(self, deadline: float) -> MSPFrame:
        state = 0
        while time.monotonic() < deadline:
            item = self.serial.read(1)
            if not item:
                continue
            value = item[0]
            if state == 0:
                state = 1 if value == ord("$") else 0
            elif state == 1:
                state = 2 if value == ord("X") else (1 if value == ord("$") else 0)
            else:
                if value not in (ord(">"), ord("!")):
                    state = 1 if value == ord("$") else 0
                    continue
                header = self._read_exact(5, deadline)
                _flags, command, size = struct.unpack("<BHH", header)
                payload = self._read_exact(size, deadline)
                received = self._read_exact(1, deadline)[0]
                expected = crc8_dvb_s2(header + payload)
                if received != expected:
                    raise MSPChecksumError(
                        f"MSPv2 {command} CRC 0x{received:02x}, expected 0x{expected:02x}"
                    )
                return MSPFrame(command, payload, value == ord("!"))
        raise MSPTimeout("No MSPv2 response")

    def request(self, command: int, payload: bytes = b"") -> bytes:
        frame_bytes = build_request(command, payload)
        last_error: Exception | None = None
        with self._lock:
            for _attempt in range(self.retries + 1):
                try:
                    self.serial.write(frame_bytes)
                    self.serial.flush()
                    deadline = time.monotonic() + self.request_timeout
                    while True:
                        frame = self._read_frame(deadline)
                        if frame.command != command:
                            continue
                        if frame.error:
                            raise MSPCommandError(f"INAV rejected MSP command {command}")
                        return frame.payload
                except (MSPTimeout, MSPChecksumError, OSError) as exc:
                    last_error = exc
                    self.serial.reset_input_buffer()
            raise MSPError(f"MSP command {command} failed: {last_error}") from last_error

    def send(self, command: int, payload: bytes = b"") -> None:
        """Send an MSPv1 input command without waiting for an acknowledgement."""
        frame_bytes = build_request(command, payload)
        with self._lock:
            try:
                written = self.serial.write(frame_bytes)
            except OSError as exc:
                raise MSPError(f"MSP command {command} write failed: {exc}") from exc
            if written != len(frame_bytes):
                raise MSPError(
                    f"MSP command {command} short write: {written}/{len(frame_bytes)} bytes"
                )

    def request_v2(
        self, command: int, payload: bytes = b"", flags: int = 0
    ) -> bytes:
        frame_bytes = build_v2_request(command, payload, flags)
        last_error: Exception | None = None
        with self._lock:
            for _attempt in range(self.retries + 1):
                try:
                    self.serial.write(frame_bytes)
                    self.serial.flush()
                    deadline = time.monotonic() + self.request_timeout
                    while True:
                        frame = self._read_v2_frame(deadline)
                        if frame.command != command:
                            continue
                        if frame.error:
                            raise MSPCommandError(f"INAV rejected MSPv2 command {command}")
                        return frame.payload
                except (MSPTimeout, MSPChecksumError, OSError) as exc:
                    last_error = exc
                    self.serial.reset_input_buffer()
            raise MSPError(f"MSPv2 command {command} failed: {last_error}") from last_error
