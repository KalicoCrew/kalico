import os
import sys

KLIPPER_SIM_DIR = os.environ.get(
    "KLIPPER_SIM_DIR", os.path.expanduser("~/Developer/klipper-sim/")
)
THIS_FORK = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

TEST_GCODE = """
G28 X Y
G1 X10 F600
G1 X0 F600
G1 X50 Y50 F12000
G1 X0 Y0 F12000
"""


def klipper_sim_available():
    return os.path.isdir(KLIPPER_SIM_DIR) and os.path.isfile(
        os.path.join(KLIPPER_SIM_DIR, "simulate_gcode.py")
    )


def run_sim(klipper_root, label):
    if not klipper_sim_available():
        raise RuntimeError(
            f"klipper-sim not found at {KLIPPER_SIM_DIR}. "
            "Install from the klipper-sim repo or set the KLIPPER_SIM_DIR "
            "env var to its checkout."
        )
    # TODO: thread TEST_GCODE through klipper-sim. The expected shape:
    #
    #   python3 <klipper-sim>/simulate_gcode.py \
    #       --klipper-root <klipper_root> \
    #       --mode vanilla \
    #       --max-velocity 500 --max-accel 5000 \
    #       <test.gcode> \
    #       -o steps.csv
    #
    # then convert per-100-µs trajectory samples into per-step times by
    # running stepcompress over the commanded position stream — or, once
    # klipper-sim grows a `--emit-steps` flag, parse that directly.
    raise NotImplementedError(
        "klipper-sim CLI invocation pending — klipper-sim currently emits "
        f"100-µs trajectory samples, not step-pulse events. See "
        f"{KLIPPER_SIM_DIR}/README.md and STATUS.md for current CSV schema; "
        "this test is a documentation placeholder until a step-event bridge "
        "(either a klipper-sim feature or a local stepcompress wrapper) "
        "exists."
    )


def main():
    if not klipper_sim_available():
        print(f"SKIP: klipper-sim not installed at {KLIPPER_SIM_DIR}")
        return 0

    mainline = run_sim("/path/to/mainline/klipper", "mainline")
    ours = run_sim(THIS_FORK, "redesign")

    assert len(mainline) == len(ours), (
        f"step count mismatch: {len(mainline)} vs {len(ours)}"
    )
    max_drift = 0
    for m, o in zip(mainline, ours):
        assert m[0] == o[0], f"axis mismatch: {m} vs {o}"
        drift_ns = abs(m[1] - o[1]) * 1000
        max_drift = max(max_drift, drift_ns)
    assert max_drift < 500, (
        f"max step-time drift {max_drift} ns exceeds 500 ns threshold"
    )
    print(f"PASS: max drift {max_drift:.1f} ns")
    return 0


if __name__ == "__main__":
    sys.exit(main())
