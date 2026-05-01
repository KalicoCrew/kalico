"""Stub configuration for non-motion MCUs in Phase 1 smoke tests.

For Phase 1, all MCUs go through the bridge without actual serial connections.
The bridge's claim_mcu stores the handle but doesn't open the port.
Non-motion MCU commands (bottom, beacon, NIS) are silently accepted
by the router but don't go anywhere.

This fixture provides the configuration context for the smoke test.
Phase 2 will add actual serial open + identify handshake for these MCUs.
"""

# Non-motion MCUs that the Trident config references but are not simulated
# in Phase 1. The bridge's claim_mcu accepts them and returns handles, but
# no serial port is opened and commands are buffered (never sent).
STUB_MCUS = {
    "bottom": {"serial": "/dev/null", "baud": 250000},
    "beacon": {"serial": "/dev/null", "baud": 250000},
    "NIS": {"serial": "/dev/null", "baud": 250000},
}


def claim_stub_mcus(bridge):
    """Claim all stub MCUs on the bridge, returning a dict of {name: handle}.

    This mirrors what klippy does during startup for secondary MCUs.
    In Phase 1 these handles exist in the router but never send data.
    """
    handles = {}
    for name, params in STUB_MCUS.items():
        h = bridge.claim_mcu(name, params["serial"], params["baud"])
        handles[name] = h
    return handles
