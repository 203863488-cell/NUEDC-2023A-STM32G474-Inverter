"""Offline robustness check for the inverter controller.

The equations intentionally mirror User/inverter_control.c.  This script does
not replace hardware THD measurements; it verifies that the selected PR gains
and harmonic-learning update remain stable across the declared plant range.
"""

from __future__ import annotations

import cmath
import math


VDC = 48.0
L1 = 1.0e-3
L2 = 1.0e-3
CAPACITANCE = 2.2e-6
OUTPUT_HZ = 50.0
SWITCHING_HZ = 100_000.0
CONTROL_HZ = 50_000.0
CROSSOVER_HZ = 100.0
LPF_HZ = 500.0
PHASE_MARGIN_REQUIRED_DEG = 60.0 + 3.0
GAIN_MARGIN_REQUIRED_DB = 6.0 + 1.0

PR_ORDERS = (1, 3, 5, 7, 9)
PR_BANDWIDTHS_HZ = (2.0, 5.0, 5.0, 5.0, 5.0)
PR_TARGET_GAINS = (20.0, 8.0, 5.0, 3.5, 2.5)
LEARNING_ORDERS = tuple(range(3, 26, 2))

ALPHA = 1.0 - math.exp(-2.0 * math.pi * LPF_HZ / CONTROL_HZ)
RESONANCE_HZ = math.sqrt(
    (L1 + L2) / (L1 * L2 * CAPACITANCE)
) / (2.0 * math.pi)


def fast_sin_cos_small(angle: float) -> tuple[float, float]:
    x2 = angle * angle
    x4 = x2 * x2
    x6 = x4 * x2
    x8 = x4 * x4
    sine = angle * (1.0 - x2 / 6.0 + x4 / 120.0 - x6 / 5040.0)
    cosine = (
        1.0
        - x2 / 2.0
        + x4 / 24.0
        - x6 / 720.0
        + x8 / 40320.0
    )
    return sine, cosine


def command_path(
    frequency_hz: float,
    load_ohm: float,
    l1_scale: float = 1.0,
    l2_scale: float = 1.0,
    c_scale: float = 1.0,
) -> complex:
    omega = 2.0 * math.pi * frequency_hz
    sample_time = 1.0 / CONTROL_HZ
    l1 = L1 * l1_scale
    l2 = L2 * l2_scale
    capacitance = CAPACITANCE * c_scale
    denominator = complex(
        load_ohm - l1 * load_ohm * capacitance * omega**2,
        (l1 + l2) * omega - l1 * l2 * capacitance * omega**3,
    )
    plant = load_ohm / denominator
    zoh_argument = 0.5 * omega * sample_time
    zoh_sine, _ = fast_sin_cos_small(zoh_argument)
    _, two_sample_average = fast_sin_cos_small(
        0.5 * omega / SWITCHING_HZ
    )
    delay_sine, delay_cosine = fast_sin_cos_small(
        omega * 1.625 * sample_time
    )
    zoh_gain = zoh_sine / zoh_argument
    delay = complex(delay_cosine, -delay_sine)
    return VDC * plant * zoh_gain * two_sample_average * delay


def proportional_filter(frequency_hz: float) -> complex:
    sine, cosine = fast_sin_cos_small(
        2.0 * math.pi * frequency_hz / CONTROL_HZ
    )
    z_inverse = complex(cosine, -sine)
    single_pole = ALPHA / (1.0 - (1.0 - ALPHA) * z_inverse)
    return single_pole * single_pole


def pr_coefficients(
    frequency_hz: float, bandwidth_hz: float
) -> tuple[float, float, float, float]:
    sample_time = 1.0 / CONTROL_HZ
    w0 = 2.0 * math.pi * frequency_hz
    wc = 2.0 * math.pi * bandwidth_hz
    prewarped = w0 / math.tan(0.5 * w0 * sample_time)
    denominator = prewarped**2 + 2.0 * wc * prewarped + w0**2
    return (
        2.0 * wc * prewarped / denominator,
        -2.0 * wc * prewarped / denominator,
        (-2.0 * prewarped**2 + 2.0 * w0**2) / denominator,
        (prewarped**2 - 2.0 * wc * prewarped + w0**2)
        / denominator,
    )


PR_COEFFICIENTS = tuple(
    pr_coefficients(order * OUTPUT_HZ, bandwidth)
    for order, bandwidth in zip(PR_ORDERS, PR_BANDWIDTHS_HZ)
)


def pr_response(frequency_hz: float, branch: int) -> complex:
    b0, b2, a1, a2 = PR_COEFFICIENTS[branch]
    sine, cosine = fast_sin_cos_small(
        2.0 * math.pi * frequency_hz / CONTROL_HZ
    )
    z_inverse = complex(cosine, -sine)
    return (b0 + b2 * z_inverse**2) / (
        1.0 + a1 * z_inverse + a2 * z_inverse**2
    )


PROPORTIONAL_GAIN = 1.0 / abs(
    command_path(CROSSOVER_HZ, 10.0)
    * proportional_filter(CROSSOVER_HZ)
)


def controller_response(frequency_hz: float, gains: list[float]) -> complex:
    response = PROPORTIONAL_GAIN * proportional_filter(frequency_hz)
    for branch, gain in enumerate(gains):
        response += gain * pr_response(frequency_hz, branch)
    return response


def desired_gain(branch: int) -> float:
    frequency_hz = PR_ORDERS[branch] * OUTPUT_HZ
    plant = command_path(frequency_hz, 10.0)
    base = PROPORTIONAL_GAIN * proportional_filter(frequency_hz)
    basis = pr_response(frequency_hz, branch)
    desired_controller = PR_TARGET_GAINS[branch] / abs(plant)
    basis_squared = abs(basis) ** 2
    projection = (base * basis.conjugate()).real
    constant = abs(base) ** 2 - desired_controller**2
    discriminant = projection**2 - basis_squared * constant
    if discriminant <= 0.0 or basis_squared <= 1.0e-12:
        return 0.0
    return min(
        2.0,
        max(0.0, (-projection + math.sqrt(discriminant)) / basis_squared),
    )


def loop_response(
    frequency_hz: float,
    gains: list[float],
    load_ohm: float,
    l1_scale: float,
    l2_scale: float,
    c_scale: float,
) -> complex:
    return command_path(
        frequency_hz, load_ohm, l1_scale, l2_scale, c_scale
    ) * controller_response(frequency_hz, gains)


def robust_margins(
    gains: list[float], all_component_corners: bool
) -> tuple[float, float]:
    minimum_phase = 180.0
    minimum_gain = 60.0
    if all_component_corners:
        corners = tuple(
            (l1, l2, capacitance)
            for l1 in (0.9, 1.1)
            for l2 in (0.9, 1.1)
            for capacitance in (0.9, 1.1)
        )
    else:
        corners = ((0.9, 0.9, 0.9), (1.0, 1.0, 1.0), (1.1, 1.1, 1.1))

    maximum_frequency = min(0.38 * RESONANCE_HZ, 1800.0)
    for load in (5.0, 10.0, 20.0):
        for l1_scale, l2_scale, c_scale in corners:
            previous = loop_response(
                20.0, gains, load, l1_scale, l2_scale, c_scale
            )
            previous_magnitude = abs(previous)
            frequency = 22.0
            while frequency <= maximum_frequency:
                loop = loop_response(
                    frequency,
                    gains,
                    load,
                    l1_scale,
                    l2_scale,
                    c_scale,
                )
                magnitude = abs(loop)
                if (
                    previous_magnitude >= 1.0 > magnitude
                    or previous_magnitude < 1.0 <= magnitude
                ):
                    phase = math.degrees(cmath.phase(loop))
                    if phase > 0.0:
                        phase -= 360.0
                    minimum_phase = min(minimum_phase, 180.0 + phase)
                if (
                    (
                        previous.imag >= 0.0 > loop.imag
                        or previous.imag < 0.0 <= loop.imag
                    )
                    and previous.real < 0.0
                    and loop.real < 0.0
                ):
                    crossing_magnitude = 0.5 * (
                        previous_magnitude + magnitude
                    )
                    minimum_gain = min(
                        minimum_gain,
                        -20.0 * math.log10(max(1.0e-9, crossing_magnitude)),
                    )
                previous = loop
                previous_magnitude = magnitude
                frequency += 2.0
    return minimum_phase, minimum_gain


def margins_pass(gains: list[float], all_component_corners: bool) -> bool:
    phase, gain = robust_margins(gains, all_component_corners)
    return (
        phase >= PHASE_MARGIN_REQUIRED_DEG
        and gain >= GAIN_MARGIN_REQUIRED_DB
    )


def tune_pr_gains() -> list[float]:
    desired = [desired_gain(branch) for branch in range(len(PR_ORDERS))]
    gains = [0.0] * len(PR_ORDERS)
    assert margins_pass(gains, False), "P-only loop does not meet margins"

    for branch, requested in enumerate(desired):
        low = 0.0
        high = requested
        gains[branch] = high
        if margins_pass(gains, False):
            low = high
        else:
            gains[branch] = 0.0
            for _ in range(8):
                middle = 0.5 * (low + high)
                gains[branch] = middle
                if margins_pass(gains, False):
                    low = middle
                else:
                    high = middle
        gains[branch] = low

    original = gains.copy()
    if not margins_pass(gains, True):
        low = 0.0
        high = 1.0
        for _ in range(10):
            middle = 0.5 * (low + high)
            gains = [gain * middle for gain in original]
            if margins_pass(gains, True):
                low = middle
            else:
                high = middle
        gains = [gain * low for gain in original]
    return gains


def closed_loop_command(
    frequency_hz: float,
    gains: list[float],
    load_ohm: float,
    l1_scale: float,
    l2_scale: float,
    c_scale: float,
) -> complex:
    plant = command_path(
        frequency_hz, load_ohm, l1_scale, l2_scale, c_scale
    )
    loop = plant * controller_response(frequency_hz, gains)
    return plant / (1.0 + loop)


def learning_bank_check(gains: list[float]) -> list[tuple[int, float, float]]:
    results: list[tuple[int, float, float]] = []
    for order in LEARNING_ORDERS:
        frequency = order * OUTPUT_HZ
        nominal = closed_loop_command(
            frequency, gains, 10.0, 1.0, 1.0, 1.0
        )
        inverse = nominal.conjugate() / (abs(nominal) ** 2 * 1.04)
        maximum_stable_rate = math.inf
        actual_models: list[complex] = []
        for load in (5.0, 10.0, 20.0):
            for l1_scale in (0.9, 1.1):
                for l2_scale in (0.9, 1.1):
                    for c_scale in (0.9, 1.1):
                        actual = closed_loop_command(
                            frequency,
                            gains,
                            load,
                            l1_scale,
                            l2_scale,
                            c_scale,
                        )
                        actual_models.append(actual)
                        normalized = inverse * actual
                        assert normalized.real > 0.0
                        maximum_stable_rate = min(
                            maximum_stable_rate,
                            2.0 * normalized.real / abs(normalized) ** 2,
                        )
        learning_rate = min(0.08, 0.25 * maximum_stable_rate)
        worst_pole = max(
            abs(1.0 - learning_rate * inverse * actual)
            for actual in actual_models
        )
        assert worst_pole < 1.0
        results.append((order, learning_rate, worst_pole))
    return results


def main() -> None:
    gains = tune_pr_gains()
    phase_margin, gain_margin = robust_margins(gains, True)
    learning = learning_bank_check(gains)
    worst_learning_pole = max(item[2] for item in learning)
    convergence_95_s = (
        math.log(0.05) / math.log(worst_learning_pole) / OUTPUT_HZ
    )

    assert phase_margin >= PHASE_MARGIN_REQUIRED_DEG
    assert gain_margin >= GAIN_MARGIN_REQUIRED_DB
    assert convergence_95_s <= 2.0

    print(f"LCL resonance: {RESONANCE_HZ:.2f} Hz")
    print(f"PR gains: {[round(value, 8) for value in gains]}")
    print(f"Minimum phase margin: {phase_margin:.2f} deg")
    print(f"Minimum gain margin: {gain_margin:.2f} dB")
    print(f"Worst learning pole: {worst_learning_pole:.6f}")
    print(f"Estimated 95% learning convergence: {convergence_95_s:.2f} s")
    print("PASS")


if __name__ == "__main__":
    main()
