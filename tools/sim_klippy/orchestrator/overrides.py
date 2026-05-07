"""Pin / SPI bus / serial path override layer.

Reads pin-overrides.toml and applies the mappings to a klippy config
text in-memory so the vendored printer.cfg can stay verbatim. We
operate at the printer.cfg-text level (not at klippy's section/option
parser) because klippy's resolver dispatches to chelper before we get
a hook in — easier to substitute the strings up front."""
import re
from pathlib import Path

try:
    import tomllib  # py 3.11+
except ImportError:
    import tomli as tomllib  # type: ignore


def _flatten(d: dict, prefix: str = "") -> dict:
    """Flatten a nested TOML dict into dot-separated top-level keys.

    TOML parses ``[mcu_main.gpio]`` as ``{"mcu_main": {"gpio": {...}}}``
    but ``apply_overrides`` expects ``{"mcu_main.gpio": {...}}``.  We
    flatten one level so callers get the compact dotted-section names.
    """
    out = {}
    for k, v in d.items():
        full_key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict) and not any(isinstance(vv, dict) for vv in v.values()):
            # leaf table — store under the dotted key
            out[full_key] = v
        elif isinstance(v, dict):
            out.update(_flatten(v, full_key))
        else:
            out[full_key] = v
    return out


def load_overrides(path):
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    return _flatten(raw)


def apply_overrides(cfg_text: str, overrides: dict) -> str:
    """Substitute real-hardware identifiers with sim equivalents.

    Replaces, in order: STM32 pin names (PG4 → gpiochip0/...), SPI bus
    names (spi1 → sim_spi0), USB serial-by-id substring matches.

    Pin and bus substitution use word-boundary regex so we don't
    accidentally rewrite "PA2" inside "PA20" or "spi1" inside "spi10".
    """
    out = cfg_text
    gpio_map = overrides.get("mcu_main.gpio", {})
    for real, sim in gpio_map.items():
        out = re.sub(rf"\b{re.escape(real)}\b", sim, out)
    spi_map = overrides.get("mcu_main.spi", {})
    for real, sim in spi_map.items():
        out = re.sub(rf"\b{re.escape(real)}\b", sim, out)
    serial_map = overrides.get("mcu_main.serial", {})
    for pattern, sim in serial_map.items():
        regex = re.escape(pattern).replace(r"\*", r"[^\s]*")
        out = re.sub(rf"/dev/serial/by-id/{regex}", sim, out)
    return out
