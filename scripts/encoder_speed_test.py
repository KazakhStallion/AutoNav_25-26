"""Utility script to run a 5 mph encoder logging test.

This script commands the robot to drive forward at the requested
speed (default 5 mph) for the requested duration (default 5 seconds)
while periodically sampling the wheel encoder counts.  The collected
data is persisted to a log file so that it can be analysed later.

The implementation mirrors the behaviour of the existing serial based
helpers that are used throughout the project.  The script uses the
Roboteq command set (``!G`` for motor commands and ``?C`` for encoder
queries) via the same serial parameters that the other utilities rely
on.  The encoder samples are augmented with timing information and a
rough on-the-fly mph estimate so that the log captures both raw counts
and derived metrics.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import serial


# Wheel/encoder parameters that already exist in the C++ control stack
# (see ``wheel_odom_pub.cpp`` and ``motor_controller.cpp`` for the
# corresponding values).
TICKS_PER_REVOLUTION = 81923
WHEEL_RADIUS_METERS = 0.205
WHEEL_CIRCUMFERENCE_METERS = 2 * math.pi * WHEEL_RADIUS_METERS
METERS_PER_TICK = WHEEL_CIRCUMFERENCE_METERS / TICKS_PER_REVOLUTION
MPH_PER_MPS = 2.2369362920544


@dataclass
class Sample:
    """Container for one encoder sample."""

    elapsed_s: float
    left_count: int
    right_count: int
    left_delta: int
    right_delta: int
    average_mph: float


class SamplingError(RuntimeError):
    """Raised when sampling aborts prematurely but partial data exists."""

    def __init__(
        self,
        message: str,
        *,
        samples: List[Sample],
        initial_left: Optional[int],
        initial_right: Optional[int],
        last_left: Optional[int],
        last_right: Optional[int],
    ) -> None:
        super().__init__(message)
        self.samples = samples
        self.initial_left = initial_left
        self.initial_right = initial_right
        self.last_left = last_left
        self.last_right = last_right


def mph_to_command_value(
    mph: float,
    *,
    max_mph: float,
    stepsize: int,
) -> int:
    """Convert the target mph to a Roboteq velocity command value.

    The ``MotorControlAPI`` sends values between ``-1000`` and ``1000`` by
    multiplying the *speed percentage* by a ``stepsize`` (10 by default).
    We replicate the same logic here but allow the caller to express the
    target as mph.  ``max_mph`` describes the velocity that corresponds to
    100% output and therefore acts as a calibration constant that can be
    tuned empirically on the robot.
    """

    if max_mph <= 0:
        raise ValueError("max_mph must be positive")

    percentage = max(min(mph / max_mph, 1.0), -1.0) * 100.0
    return int(round(percentage * stepsize))


def initialise_serial(port: str, *, baudrate: int = 115200, timeout: float = 0.2) -> serial.Serial:
    """Create and configure the serial connection to the motor controller."""

    ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    # Ensure the controller is in closed-loop speed mode, mirroring other scripts.
    ser.write(b"!MG\r")
    return ser


_ENCODER_PATTERN = re.compile(r"(-?\d+)")


def read_encoder_count(ser: serial.Serial, channel: int) -> int:
    """Query the encoder count for the requested channel."""

    command = f"?C {channel}\r".encode("ascii")
    print(f"Sending command: {command}")  # Test
    ser.reset_input_buffer()
    ser.write(command)

    deadline = time.time() + (ser.timeout or 0.2) * 3
    print(f"Waiting for response until {deadline:.3f}") # Test
    while time.time() < deadline:
        response = ser.readline().decode("ascii", errors="ignore").strip()
        print(f"Received response: {response}")  # Test
        if not response:
            continue

        match = _ENCODER_PATTERN.search(response)
        if match:
            return int(match.group(1))

    raise RuntimeError(f"Timed out reading encoder value for channel {channel}")


def send_drive_command(ser: serial.Serial, command_value: int) -> None:
    """Send a forward drive command using the same format as ``MotorControlAPI``."""

    left_command = f"!G 1 {-command_value}\r".encode("ascii")
    right_command = f"!G 2 {command_value}\r".encode("ascii")
    ser.write(left_command)
    ser.write(right_command)


def stop_motors(ser: serial.Serial) -> None:
    """Command both motors to stop."""

    ser.write(b"!G 1 0\r")
    ser.write(b"!G 2 0\r")


def counts_to_mph(delta_counts: float, dt: float) -> float:
    """Convert encoder delta counts over a time delta to mph."""

    if dt <= 0:
        return 0.0
    meters_per_second = delta_counts * METERS_PER_TICK / dt
    return meters_per_second * MPH_PER_MPS


def collect_samples(
    ser: serial.Serial,
    *,
    duration_s: float,
    sample_interval_s: float,
) -> Tuple[List[Sample], int, int, int, int]:
    """Collect encoder samples for the supplied duration."""

    left_channel = 2
    right_channel = 1

    samples: List[Sample] = []
    initial_left: Optional[int] = None
    initial_right: Optional[int] = None
    left_prev: Optional[int] = None
    right_prev: Optional[int] = None
    last_sample_time: Optional[float] = None

    try:
        initial_left = left_prev = read_encoder_count(ser, left_channel)
        print(f"Initial left encoder count: {initial_left}")  # Test
        initial_right = right_prev = read_encoder_count(ser, right_channel)
        print(f"Initial right encoder count: {initial_right}")  # Test
        start_time = time.perf_counter()
        last_sample_time = start_time
        next_sample_time = start_time + sample_interval_s

        while True:
            now = time.perf_counter()
            elapsed = now - start_time
            print(f"Elapsed time: {elapsed:.3f}s")  # Test
            if elapsed >= duration_s:
                break

            sleep_time = next_sample_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_sample_time += sample_interval_s

            sample_time = time.perf_counter()
            left_current = read_encoder_count(ser, left_channel)
            print(f"Sampled left encoder count: {left_current}")  # Test
            right_current = read_encoder_count(ser, right_channel)
            print(f"Sampled right encoder count: {right_current}")  # Test

            dt = sample_time - last_sample_time
            left_delta = left_current - (left_prev or left_current)
            right_delta = right_current - (right_prev or right_current)

            # Use the average absolute delta to avoid sign issues when the
            # encoder counts use opposite conventions for each wheel.
            avg_delta = (abs(left_delta) + abs(right_delta)) / 2.0
            mph = counts_to_mph(avg_delta, dt)
            print(
                f"Sampled deltas: left={left_delta} right={right_delta} "
                f"average_mph={mph:.3f}"
            ) # Test

            samples.append(
                Sample(
                    elapsed_s=sample_time - start_time,
                    left_count=left_current,
                    right_count=right_current,
                    left_delta=left_delta,
                    right_delta=right_delta,
                    average_mph=mph,
                )
            )

            left_prev = left_current
            right_prev = right_current
            last_sample_time = sample_time

        final_left = read_encoder_count(ser, left_channel)
        print(f"Final left encoder count: {final_left}")
        final_right = read_encoder_count(ser, right_channel)    # Test
        print(f"Final right encoder count: {final_right}")  # Test

        return samples, initial_left, initial_right, final_left, final_right
    except RuntimeError as exc:
        raise SamplingError(
            str(exc),
            samples=samples,
            initial_left=initial_left,
            initial_right=initial_right,
            last_left=left_prev,
            last_right=right_prev,
        ) from exc


def write_log(
    path: Path,
    *,
    samples: Iterable[Sample],
    initial_left: int,
    initial_right: int,
    final_left: int,
    final_right: int,
    duration_s: float,
    speed_mph: float,
    max_mph: float,
    command_value: int,
    notes: Optional[Iterable[str]] = None,
) -> None:
    """Persist the collected data to a log file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().isoformat(timespec="seconds")

    total_left_delta = final_left - initial_left
    total_right_delta = final_right - initial_right
    avg_delta = (abs(total_left_delta) + abs(total_right_delta)) / 2.0
    distance_m = avg_delta * METERS_PER_TICK
    average_mph = counts_to_mph(avg_delta, duration_s)

    header = (
        f"Encoder speed test log ({timestamp})\n"
        f"Duration: {duration_s:.2f}s\n"
        f"Commanded speed: {speed_mph:.2f} mph (max mph calibration {max_mph:.2f})\n"
        f"Motor command value: {command_value}\n"
        f"Initial encoder counts  L={initial_left}  R={initial_right}\n"
        f"Final encoder counts    L={final_left}  R={final_right}\n"
        f"Total delta counts      L={total_left_delta}  R={total_right_delta}\n"
        f"Approx distance travelled: {distance_m:.3f} m\n"
        f"Approx average speed: {average_mph:.3f} mph\n"
    )

    if notes:
        for note in notes:
            header += f"Note: {note}\n"

    header += (
        "\n"
        "elapsed_s,left_count,right_count,left_delta,right_delta,average_mph\n"
    )

    with path.open("w", encoding="utf-8") as log_file:
        log_file.write(header)
        for sample in samples:
            log_file.write(
                f"{sample.elapsed_s:.3f},{sample.left_count},{sample.right_count},"
                f"{sample.left_delta},{sample.right_delta},{sample.average_mph:.3f}\n"
            )


def run_test(args: argparse.Namespace) -> None:
    ser = initialise_serial(args.port, timeout=args.serial_timeout)
    command_value = mph_to_command_value(
        args.speed_mph,
        max_mph=args.max_mph,
        stepsize=args.stepsize,
    )

    print(
        f"Starting speed test: {args.speed_mph:.2f} mph for {args.duration:.2f}s"
        f" (command value {command_value})"
    )

    samples: List[Sample] = []
    initial_left = initial_right = final_left = final_right = 0
    notes: List[str] = []

    try:
        send_drive_command(ser, command_value)
        try:
            (
                samples,
                initial_left,
                initial_right,
                final_left,
                final_right,
            ) = collect_samples(
                ser,
                duration_s=args.duration,
                sample_interval_s=args.sample_interval,
            )
        except SamplingError as exc:
            notes.append("Sampling ended early: " + str(exc))
            samples = exc.samples
            if exc.initial_left is not None:
                initial_left = exc.initial_left
            if exc.initial_right is not None:
                initial_right = exc.initial_right
            final_left = exc.last_left if exc.last_left is not None else initial_left
            final_right = exc.last_right if exc.last_right is not None else initial_right
            print(
                "Sampling aborted early due to serial timeout; log will contain"
                " partial data."
            )
    finally:
        stop_motors(ser)

    write_log(
        args.log_file,
        samples=samples,
        initial_left=initial_left,
        initial_right=initial_right,
        final_left=final_left,
        final_right=final_right,
        duration_s=args.duration,
        speed_mph=args.speed_mph,
        max_mph=args.max_mph,
        command_value=command_value,
        notes=notes,
    )

    print(f"Test complete. Log written to {args.log_file}")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Encoder speed logging test")
    parser.add_argument("port", help="Serial port connected to the motor controller")
    parser.add_argument(
        "--speed-mph",
        type=float,
        default=5.0,
        help="Target forward speed in mph (default: 5)",
    )
    parser.add_argument(
        "--max-mph",
        type=float,
        default=10.0,
        help=(
            "Calibration constant that maps 100%% controller output to mph. "
            "Adjust to match the robot's measured top speed."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="How long to drive forward in seconds (default: 5)",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=0.1,
        help="Time between encoder samples in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--stepsize",
        type=int,
        default=10,
        help="Controller scaling factor used by MotorControlAPI (default: 10)",
    )
    parser.add_argument(
        "--serial-timeout",
        type=float,
        default=0.2,
        help="Serial read timeout in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=Path("~/logs/encoder_speed_test.log"),
        help="Where to write the output log (default: /logs/encoder_speed_test.log)",
    )

    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> None:
    args = parse_args(argv)
    run_test(args)


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print("Test interrupted by user")