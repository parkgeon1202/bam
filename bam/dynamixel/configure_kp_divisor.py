# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""
Identify the internal firmware "Kp divisor" of a Dynamixel XH-540 (or any
other X430-generation X-series servo sharing its control table) by directly
driving the motor and reading back its own duty-cycle command -- no
oscilloscope, no post-hoc log processing.

Firmware model (see bam/dynamixel/actuator.py, XL330Actuator):

    raw_pwm = kp_firmware * error_count / KP_DIVISOR      (before saturation)
    duty    = raw_pwm / PWM_LIMIT

=>  KP_DIVISOR = kp_firmware * error_count / (duty * PWM_LIMIT)

For each firmware Kp in --kps, this commands a small triangle-wave goal
position (small enough to stay out of PWM saturation) and, using
DynamixelXH540's Indirect Address status block, reads back
{hardware_error_status, present pwm, present position} in a single bus
transaction per sample. Each unsaturated sample gives one estimate of
KP_DIVISOR; a slope-through-origin least-squares fit over all samples for a
given Kp is logged live, and a final pooled estimate is printed at the end.

Usage:
    uv run -m bam.dynamixel.configure_kp_divisor --port /dev/ttyUSB0
"""

import argparse
import time

import numpy as np

from .dynamixel import DynamixelXH540, XH540_RESOLUTION

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--id", type=int, default=1)
arg_parser.add_argument(
    "--kps", type=float, nargs="+",
    default=[50, 100, 200, 300, 400, 600, 800],
    help="Firmware Kp values to sweep",
)
arg_parser.add_argument(
    "--step-deg", type=float, default=2.0,
    help="Initial triangle-wave half-amplitude [deg] for the first Kp -- "
    "auto-calibrated per Kp afterwards, see --target-duty",
)
arg_parser.add_argument(
    "--max-step-deg", type=float, default=45.0,
    help="Safety cap on the auto-calibrated amplitude",
)
arg_parser.add_argument(
    "--target-duty", type=float, default=0.3,
    help="Amplitude is auto-tuned so |duty| lands near this fraction of PWM_LIMIT -- "
    "too small and Present PWM is dominated by quantization noise (1 LSB "
    "~= 0.113%%), too big and it saturates",
)
arg_parser.add_argument("--period", type=float, default=1.0, help="Triangle-wave period [s]")
arg_parser.add_argument("--duration", type=float, default=4.0, help="Sampling time per Kp [s]")
arg_parser.add_argument("--dt", type=float, default=0.01, help="Sampling period [s]")
arg_parser.add_argument(
    "--max-duty", type=float, default=0.9,
    help="Discard samples where |duty| exceeds this (near/at saturation)",
)
arg_parser.add_argument(
    "--min-error-deg", type=float, default=0.5,
    help="Discard samples with too small a position error (noisy)",
)
arg_parser.add_argument(
    "--goal-source", choices=["readback", "local"], default="readback",
    help="'readback' (default, correct): compute error against "
    "status['goal_position_readback'], the goal the servo actually had "
    "when it produced this same duty/position snapshot. 'local': use the "
    "goal_position value we just wrote in Python instead -- reproduces the "
    "write/read timing-lag bug this flag exists to A/B against, for "
    "confirming that bug was the cause of the earlier kp-dependent drift.",
)
args = arg_parser.parse_args()

COUNTS_PER_RAD = XH540_RESOLUTION / (2 * np.pi)


def triangle_wave(t: float, amplitude: float, period: float) -> float:
    """Triangle wave in [-amplitude, amplitude], starting at 0 and rising."""
    phase = (t / period) % 1.0
    if phase < 0.5:
        return -amplitude + 4 * amplitude * phase
    return 3 * amplitude - 4 * amplitude * phase


def calibrate_amplitude(
    motor, kp: int, initial_amplitude: float, target_duty: float, max_amplitude: float,
    probe_duration: float = 1.0, max_iters: int = 5,
) -> float:
    """Find a triangle-wave amplitude for this Kp that drives peak |duty|
    close to target_duty. Needed because raw_pwm ~= kp*error/DIVISOR scales
    with Kp: a fixed amplitude is either buried in Present PWM's
    ~0.113%/count quantization noise at low Kp, or saturated at high Kp.

    Probes with the target continuously MOVING (not a static hold-and-wait):
    a P controller with no external disturbance (e.g. a bare motor, no
    arm/gravity load) drives a *static* target's error to ~0 once settled,
    which would always read duty~=0 regardless of amplitude. Chasing a
    moving target instead measures the velocity-lag tracking error, which is
    nonzero purely from the system's response bandwidth -- no disturbance
    torque required, so this works identically with or without a load
    attached to the horn.
    """
    amplitude = initial_amplitude
    peak_duty = 0.0
    for _ in range(max_iters):
        peak_duty = 0.0
        start = time.time()
        while time.time() - start < probe_duration:
            t = time.time() - start
            goal = triangle_wave(t, amplitude, args.period)
            motor.write_targets(goal, p_gain=kp, torque_enable=True)
            status = motor.read_status()
            peak_duty = max(peak_duty, abs(status["load"]))
            time.sleep(args.dt)

        if peak_duty < 1e-3:
            amplitude *= 3.0  # no measurable response yet -- jump up
        else:
            scale = float(np.clip(target_duty / peak_duty, 0.3, 3.0))  # damped correction
            amplitude *= scale
        amplitude = float(np.clip(amplitude, np.deg2rad(0.5), max_amplitude))

        if 0.7 * target_duty <= peak_duty <= 1.3 * target_duty:
            break

    return amplitude


def fit_divisor(xs: list, ys: list) -> float:
    """Slope-through-origin least squares: ys ~= xs / divisor.

    xs = kp_firmware * error_count, ys = duty * pwm_limit (i.e. raw_pwm).
    slope = sum(x*y) / sum(x*x) estimates 1 / KP_DIVISOR.
    """
    x = np.asarray(xs)
    y = np.asarray(ys)
    slope = float(np.dot(x, y) / np.dot(x, x))
    return 1.0 / slope


def main():
    motor = DynamixelXH540(args.port, args.id)
    max_amplitude = np.deg2rad(args.max_step_deg)
    min_error_count = np.deg2rad(args.min_error_deg) * COUNTS_PER_RAD

    # Disable trajectory smoothing: with it on, a goal_position write isn't
    # the PID's actual setpoint, which breaks duty=kp*error/DIVISOR (see
    # DynamixelXH540.set_profile docstring).
    motor.set_profile(velocity=0, acceleration=0)

    # Force pure P control: a nonzero D gain adds a -Kd*velocity term to
    # raw_pwm, which contaminates every sample taken while the motor is
    # moving (see DynamixelXH540.set_d_gain docstring) -- this is the
    # leading suspect for the Kp-dependent divisor drift we were seeing.
    motor.set_i_gain(0)
    motor.set_d_gain(0)
    p, i, d = motor.read_pid_gains()
    print(f"Connected. PWM limit: {motor.pwm_limit}  PID gains read back: P={p} I={i} D={d}")
    if i != 0 or d != 0:
        raise RuntimeError(f"I/D gain did not actually get set to 0 (read back I={i}, D={d})")

    all_x, all_y = [], []
    amplitude_guess = np.deg2rad(args.step_deg)
    prev_kp = None

    try:
        for kp in args.kps:
            kp = int(kp)

            # raw_pwm ~= kp*error/DIVISOR -> amplitude for a similar duty
            # scales like 1/kp; carry that over from the last Kp to
            # converge faster before fine-tuning against the real motor.
            if prev_kp is not None:
                amplitude_guess *= prev_kp / kp
            amplitude = calibrate_amplitude(
                motor, kp, amplitude_guess, args.target_duty, max_amplitude
            )
            print(f"kp={kp:6d}  calibrated amplitude = {np.rad2deg(amplitude):.2f} deg")
            amplitude_guess = amplitude
            prev_kp = kp

            xs, ys = [], []
            start = time.time()
            while time.time() - start < args.duration:
                t = time.time() - start
                goal = triangle_wave(t, amplitude, args.period)
                motor.write_targets(goal, p_gain=kp, torque_enable=True)

                status = motor.read_status()
                if status["hardware_error_status"]:
                    raise RuntimeError(
                        f"Hardware error status = {status['hardware_error_status']:#04x}, aborting"
                    )

                # Use the servo's own echoed goal_position, not the local
                # `goal` we just wrote: it comes from the exact same status
                # snapshot as `duty`/`position`, so it's guaranteed to be the
                # goal that actually produced this duty -- `goal` itself may
                # still be one write/read round-trip ahead of what the
                # firmware had already applied when it computed `duty`.
                # --goal-source local reproduces that lag on purpose, for
                # A/B comparison against this fix.
                reference_goal = goal if args.goal_source == "local" else status["goal_position_readback"]
                error_count = (reference_goal - status["position"]) * COUNTS_PER_RAD
                duty = status["load"]

                if abs(duty) < args.max_duty and abs(error_count) > min_error_count:
                    xs.append(kp * error_count)
                    ys.append(duty * motor.pwm_limit)

                time.sleep(args.dt)

            if len(xs) >= 5:
                divisor = fit_divisor(xs, ys)
                print(f"kp={kp:6d}  n={len(xs):4d}  KP_DIVISOR ≈ {divisor:8.2f}")
                all_x.extend(xs)
                all_y.extend(ys)
            else:
                print(f"kp={kp:6d}  not enough unsaturated samples ({len(xs)}), skipping")

    finally:
        motor.write_targets(0.0, torque_enable=False)

    if all_x:
        overall = fit_divisor(all_x, all_y)
        print()
        print(f"Pooled estimate over {len(all_x)} samples: KP_DIVISOR ≈ {overall:.2f}")
        print("(paste this into XH540_KP_DIVISOR in bam/dynamixel/actuator.py)")
    else:
        print("No usable samples collected.")


if __name__ == "__main__":
    main()
