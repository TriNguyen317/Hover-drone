from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from flight_controller.config import AppConfig, ConfigError, load_config
from flight_controller.inav import INAVClient
from flight_controller.msp import MSPError, MSPTransport


CONFIRMATION = "RESET DISARMED FC"
BIND_CONFIRMATION = "RESET FC AND BIND RP4TD"


def open_transport(config: AppConfig) -> MSPTransport:
    return MSPTransport(
        config.serial.port,
        config.serial.baud,
        request_timeout=config.serial.request_timeout_s,
        retries=config.serial.retries,
    )


def is_failsafe(modes: set[str]) -> bool:
    return any("FAILSAFE" in mode.upper() for mode in modes)


def read_safety_state(vehicle: INAVClient, config: AppConfig) -> tuple[list[int], set[str]]:
    rc = vehicle.rc_channels()
    modes = vehicle.active_modes()
    takeover_index = config.safety.takeover_channel - 1

    if "ARM" in modes:
        raise RuntimeError("INAV đang ARM; từ chối reboot")
    if len(rc) <= takeover_index:
        raise RuntimeError(
            f"MSP_RC không có CH{config.safety.takeover_channel}; từ chối reboot"
        )
    if rc[takeover_index] > config.safety.takeover_off_pwm:
        raise RuntimeError(
            f"CH{config.safety.takeover_channel} takeover đang ON "
            f"({rc[takeover_index]} > {config.safety.takeover_off_pwm}); hãy tắt CH này"
        )
    return rc, modes


def format_rc(rc: list[int]) -> str:
    return " ".join(f"CH{index}={value}" for index, value in enumerate(rc, start=1))


def reconnect(config: AppConfig, timeout_s: float) -> tuple[MSPTransport, INAVClient]:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        transport: MSPTransport | None = None
        try:
            transport = open_transport(config)
            vehicle = INAVClient(transport)
            vehicle.api_version()
            return transport, vehicle
        except (MSPError, OSError) as exc:
            last_error = exc
            if transport is not None:
                transport.close()
            time.sleep(0.5)
    raise RuntimeError(f"INAV không kết nối lại trong {timeout_s:.1f}s: {last_error}")


def read_cli_output(serial_port: object, timeout_s: float = 2.0) -> str:
    deadline = time.monotonic() + timeout_s
    last_data = time.monotonic()
    result = bytearray()
    while time.monotonic() < deadline:
        waiting = int(getattr(serial_port, "in_waiting", 0))
        chunk = serial_port.read(max(1, waiting))  # type: ignore[attr-defined]
        if chunk:
            result.extend(chunk)
            last_data = time.monotonic()
        elif result and time.monotonic() - last_data >= 0.25:
            break
    return result.decode("utf-8", errors="replace")


def enter_rp4td_bind_mode(transport: MSPTransport) -> str:
    """Ask INAV to send the CRSF RX_BIND command to the RP4TD."""
    serial_port = transport.serial
    entered_cli = False
    try:
        serial_port.reset_input_buffer()
        serial_port.write(b"#\r\n")
        serial_port.flush()
        banner = read_cli_output(serial_port)
        if "#" not in banner and "CLI" not in banner.upper():
            raise RuntimeError(
                "Không vào được INAV CLI trên cổng MSP; không gửi bind_rx"
            )
        entered_cli = True

        serial_port.write(b"bind_rx\r\n")
        serial_port.flush()
        response = read_cli_output(serial_port)
        lowered = response.lower()
        if "unknown command" in lowered or "parse error" in lowered:
            raise RuntimeError("INAV target này không hỗ trợ lệnh bind_rx")
        return banner + response
    finally:
        if entered_cli:
            # bind_rx is an immediate command and does not change saved FC
            # configuration. Exit restores MSP service without issuing save.
            serial_port.write(b"exit\r\n")
            serial_port.flush()
            read_cli_output(serial_port, timeout_s=1.0)


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(
        description="Reboot INAV an toàn rồi kiểm tra receiver/RC; không ARM và không gửi MSP RC"
    )
    parser.add_argument("--config", default="config.real.json")
    parser.add_argument("--reconnect-timeout", type=float, default=15.0)
    parser.add_argument("--monitor-seconds", type=float, default=5.0)
    parser.add_argument(
        "--bind-rp4td",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "sau reboot, gửi INAV bind_rx cho RadioMaster RP4TD CRSF "
            "(mặc định: bật; dùng --no-bind-rp4td để chỉ reboot FC)"
        ),
    )
    args = parser.parse_args()

    if args.reconnect_timeout < 3.0 or args.monitor_seconds < 1.0:
        parser.error("reconnect-timeout phải >= 3s và monitor-seconds phải >= 1s")

    try:
        config = load_config(Path(args.config))
        with open_transport(config) as transport:
            vehicle = INAVClient(transport)
            api = vehicle.api_version()
            variant = vehicle.fc_variant()
            version = vehicle.fc_version()
            rc, modes = read_safety_state(vehicle, config)
            print(
                f"FC={variant} {'.'.join(map(str, version))} | "
                f"MSP={api.protocol}, API={api.major}.{api.minor}"
            )
            print(f"DISARMED | modes={sorted(modes)}")
            print(f"RC before reboot: {format_rc(rc)}")
            print("Yêu cầu: drone ở mặt đất, tháo cánh, tay ga thấp và CH takeover OFF.")
            confirmation = BIND_CONFIRMATION if args.bind_rp4td else CONFIRMATION
            if input(f"Nhập chính xác '{confirmation}' để tiếp tục: ").strip() != confirmation:
                print("Đã hủy; không gửi lệnh reboot.")
                return 2

            # Re-read immediately before the state-changing command so the
            # decision does not rely on the earlier snapshot.
            read_safety_state(vehicle, config)
            print("Đang gửi MSP_REBOOT (normal reboot, không vào bootloader)...", flush=True)
            vehicle.reboot()

        transport, vehicle = reconnect(config, args.reconnect_timeout)
        with transport:
            api = vehicle.api_version()
            variant = vehicle.fc_variant()
            version = vehicle.fc_version()
            print(
                f"INAV đã kết nối lại: {variant} {'.'.join(map(str, version))} | "
                f"API={api.major}.{api.minor}"
            )

            read_safety_state(vehicle, config)
            if args.bind_rp4td:
                print(
                    "Mở Tools -> ExpressLRS trên tay cầm và chuẩn bị chọn [Bind].\n"
                    "Giữ drone DISARMED, tay ga thấp và CH takeover OFF."
                )
                input("Nhấn Enter để INAV gửi lệnh bind_rx tới RP4TD: ")
                read_safety_state(vehicle, config)
                cli_output = enter_rp4td_bind_mode(transport)
                print("INAV đã nhận lệnh bind_rx qua CLI.")
                if "bind_rx" not in cli_output.lower():
                    print(
                        "[WARN] CLI không echo tên lệnh; hãy xác nhận LED receiver "
                        "đang nháy kép.",
                        file=sys.stderr,
                    )

        if args.bind_rp4td:
            print(
                "Receiver phải nháy kép. Bây giờ chọn [Bind] trên ExpressLRS Lua; "
                "LED sáng ổn định là đã bind."
            )
            input("Sau khi LED sáng ổn định, nhấn Enter để kiểm tra RC/failsafe: ")

        transport, vehicle = reconnect(config, args.reconnect_timeout)
        with transport:
            deadline = time.monotonic() + args.monitor_seconds
            sample = 0
            clean_samples = 0
            while time.monotonic() < deadline:
                rc, modes = read_safety_state(vehicle, config)
                sample += 1
                failsafe = is_failsafe(modes)
                clean_samples = 0 if failsafe else clean_samples + 1
                print(
                    f"RX {sample:02d} | failsafe={'YES' if failsafe else 'NO'} | "
                    f"{format_rc(rc)}",
                    flush=True,
                )
                time.sleep(0.5)

            if clean_samples < 3:
                print(
                    "[FAIL] Chưa có 3 mẫu RC liên tiếp không FAILSAFE; chưa được bay.",
                    file=sys.stderr,
                )
                return 3

        if args.bind_rp4td:
            print("[PASS] FC reboot, RP4TD bind và đường RC được xác nhận.")
        else:
            print("[PASS] FC reboot thành công và INAV không báo receiver failsafe.")
            print("RP4TD bind đã được bỏ qua bởi --no-bind-rp4td.")
        return 0
    except (ConfigError, MSPError, OSError, RuntimeError) as exc:
        print(f"[ABORT] {exc}", file=sys.stderr)
        print("Không ARM và không có lệnh motor/MSP RC nào được gửi.", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nĐã hủy; không gửi thêm lệnh.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
