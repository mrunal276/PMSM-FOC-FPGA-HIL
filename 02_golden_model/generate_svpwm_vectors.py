"""
Generate deterministic SVPWM golden-reference test vectors.

The frozen svpwm.py is imported only; it is not modified.
CSV columns:
v_alpha, v_beta, expected_sector, expected_duty_a,
expected_duty_b, expected_duty_c
"""

import csv
import math
import random
from pathlib import Path

from svpwm import svpwm_fixed


OUTPUT_FILE = Path(__file__).resolve().parent / "svpwm_testvectors.csv"
RANDOM_SEED = 20261028
RANDOM_CASES = 2000


def to_q15(value):
    """Convert normalized input to signed Q1.15."""
    value = max(-1.0, min(32767.0 / 32768.0, value))
    return max(-32768, min(32767, round(value * 32768.0)))


def calculate_expected(alpha, beta):
    result = svpwm_fixed(alpha, beta)
    return (
        int(result.sector),
        int(result.Ta_q15),
        int(result.Tb_q15),
        int(result.Tc_q15),
    )


def main():
    vectors = []

    # Zero and axis-aligned inputs.
    special_inputs = [
        (0, 0),
        (32767, 0),
        (-32768, 0),
        (0, 32767),
        (0, -32768),
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
    ]

    vectors.extend(special_inputs)

    # Test all six sector boundaries, just below/at/above each boundary.
    # Multiple magnitudes exercise low modulation and overmodulation.
    amplitudes = [0.01, 0.10, 0.30, 0.50, 0.80, 1.00, 1.30, 1.80]
    angle_offsets = [-0.001, 0.0, 0.001]

    for boundary in range(6):
        boundary_angle = boundary * math.pi / 3.0

        for amplitude in amplitudes:
            for offset in angle_offsets:
                angle = boundary_angle + offset
                alpha = to_q15(amplitude * math.cos(angle))
                beta = to_q15(amplitude * math.sin(angle))
                vectors.append((alpha, beta))

    # Include inputs close to signed Q1.15 limits.
    for alpha in [-32768, -32767, -1, 0, 1, 32766, 32767]:
        for beta in [-32768, -32767, -1, 0, 1, 32766, 32767]:
            vectors.append((alpha, beta))

    # Reproducible random vectors across the full signed input range.
    rng = random.Random(RANDOM_SEED)
    for _ in range(RANDOM_CASES):
        alpha = rng.randint(-32768, 32767)
        beta = rng.randint(-32768, 32767)
        vectors.append((alpha, beta))

    # Remove duplicate input pairs while preserving order.
    vectors = list(dict.fromkeys(vectors))

    with OUTPUT_FILE.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)

        for alpha, beta in vectors:
            sector, duty_a, duty_b, duty_c = calculate_expected(alpha, beta)
            writer.writerow(
                [alpha, beta, sector, duty_a, duty_b, duty_c]
            )

    print("SVPWM golden vectors generated.")
    print(f"Output file : {OUTPUT_FILE}")
    print(f"Vector count: {len(vectors)}")
    print("Golden model: svpwm_fixed()")
    print("Seed        :", RANDOM_SEED)


if __name__ == "__main__":
    main()