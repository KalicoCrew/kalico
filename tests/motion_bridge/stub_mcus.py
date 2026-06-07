STUB_MCUS = {
    "bottom": {"serial": "/dev/null", "baud": 250000},
    "beacon": {"serial": "/dev/null", "baud": 250000},
    "NIS": {"serial": "/dev/null", "baud": 250000},
}


def claim_stub_mcus(bridge):
    handles = {}
    for name, params in STUB_MCUS.items():
        h = bridge.claim_mcu(name, params["serial"], params["baud"])
        handles[name] = h
    return handles
