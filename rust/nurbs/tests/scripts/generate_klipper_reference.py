#!/usr/bin/env python3
"""Generate Klipper-shaped trajectory reference for convolve cross-check.

Uses scipy.integrate.quad for numerical convolution against bleeding-edge-v2's
smooth_zv kernel applied to a synthetic trajectory. The Rust convolve oracle
asserts agreement to 1e-4 (numerical-quadrature tolerance, not exact).

The kernel coefficients are stored in absolute monomial form (Σ c_i * t^i)
around t=0 — same convention as Klipper's `init_smoother` output. The Rust
test converts them to the Pascal-shifted basis used by
`PiecewisePolynomialKernel`.

Run with:
    pip install scipy
    python rust/nurbs/tests/scripts/generate_klipper_reference.py > rust/nurbs/tests/data/klipper_smooth_zv_reference.json
"""

import json


def get_zv_smoother_local(shaper_freq):
    """Reproduce bleeding-edge-v2 shaper_defs.get_zv_smoother for shaper_freq.

    Avoids importing the whole klippy machinery (klippy isn't pip-installable
    in this repo); the coefficients are the same."""
    raw_coeffs = [
        -118.4265334338076,
        5.861885495127615,
        29.52796003014231,
        -1.465471373781904,
        0.01966833207740377,
    ]
    smooth_time = 0.8025 / shaper_freq
    inv_t = 1.0 / smooth_time
    inv_t_n = inv_t
    n = len(raw_coeffs)
    c = [0.0] * n
    for i in range(n - 1, -1, -1):
        c[n - i - 1] = raw_coeffs[i] * inv_t_n
        inv_t_n *= inv_t
    return c, smooth_time


def main():
    coeffs, t_sm = get_zv_smoother_local(30.0)
    import scipy.integrate as si

    a = 100.0  # input acceleration
    t_end = 0.5  # input duration

    def x_input(t):
        return 0.5 * a * t * t if 0 <= t <= t_end else 0.0

    def kernel(t):
        if abs(t) > t_sm / 2:
            return 0.0
        return sum(c * t**i for i, c in enumerate(coeffs))

    samples = []
    for sample_t in [0.05, 0.1, 0.2, 0.3, 0.4, 0.45]:
        s_lo = max(0.0, sample_t - t_sm / 2)
        s_hi = min(t_end, sample_t + t_sm / 2)
        if s_lo >= s_hi:
            samples.append({"T": sample_t, "value": 0.0})
            continue
        y, _ = si.quad(
            lambda s, sample_t=sample_t: x_input(s) * kernel(sample_t - s),
            s_lo,
            s_hi,
        )
        samples.append({"T": sample_t, "value": y})

    out = {
        "kernel_coeffs": list(coeffs),
        "kernel_t_sm": t_sm,
        "input_accel": a,
        "input_t_end": t_end,
        "samples": samples,
    }
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
