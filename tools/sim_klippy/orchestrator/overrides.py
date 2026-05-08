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


_SECTION_HEADER_RE = re.compile(r"^\s*\[([^\]]+)\]\s*$")


def _inject_section_keys(cfg_text: str, section: str, kv: dict) -> str:
    """Insert key=value pairs into an existing klippy [section] block.

    Only inserts keys that are not already present in the section. The
    section must already exist; if it doesn't, the injection is a no-op
    (callers can rely on the section coming from the source printer.cfg).

    Klippy's config parser tolerates blank lines and comments inside a
    section, so we insert immediately after the section header line.
    """
    lines = cfg_text.splitlines(keepends=True)
    out_lines = []
    in_target = False
    section_body_lines: list = []
    target_seen = False
    existing_keys: set = set()

    def _flush_with_inject(buf, hdr_idx_unused):
        # Append injection lines for any missing keys, then the body.
        injected = []
        for k, v in kv.items():
            if k.lower() not in existing_keys:
                injected.append(f"{k}: {v}\n")
        if injected and buf and not buf[0].endswith("\n"):
            buf[0] = buf[0] + "\n"
        return injected + buf

    i = 0
    while i < len(lines):
        line = lines[i]
        m = _SECTION_HEADER_RE.match(line.rstrip("\n"))
        if m:
            # Closing previous section — if it was the target, flush with
            # injections appended after the header.
            if in_target:
                # We're moving out of target section. Inject and flush.
                out_lines.extend(_flush_with_inject(section_body_lines, None))
                section_body_lines = []
                existing_keys = set()
                in_target = False
            section_name = m.group(1).strip()
            if section_name == section and not target_seen:
                target_seen = True
                in_target = True
                out_lines.append(line)
                i += 1
                continue
            out_lines.append(line)
            i += 1
            continue
        if in_target:
            # Track existing keys (ignore comments / blank lines).
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                # klippy options use "key: value" or "key = value".
                key_match = re.match(r"^([A-Za-z0-9_]+)\s*[:=]", stripped)
                if key_match:
                    existing_keys.add(key_match.group(1).lower())
            section_body_lines.append(line)
            i += 1
            continue
        out_lines.append(line)
        i += 1

    if in_target:
        out_lines.extend(_flush_with_inject(section_body_lines, None))

    return "".join(out_lines)


def apply_overrides(cfg_text: str, overrides: dict) -> str:
    """Substitute real-hardware identifiers with sim equivalents.

    Replaces, in order: STM32 pin names (PG4 → gpiochip0/...), SPI bus
    names (spi1 → sim_spi0), USB serial-by-id substring matches.

    Pin and bus substitution use word-boundary regex so we don't
    accidentally rewrite "PA2" inside "PA20" or "spi1" inside "spi10".

    Per-section ``config_inject`` tables (keyed as ``<section>.config_inject``
    after dotted-key flattening) inject ``key: value`` pairs into the
    matching klippy section without disturbing existing keys. This is
    used to flip headless-only options — e.g. ``[beacon].
    skip_firmware_version_check = True`` — that a real user would set in
    their own printer.cfg but the vendored config doesn't carry.
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
    # Apply per-section config_inject tables. Dotted-key form after
    # flattening: ``<section>.config_inject``.
    for key, table in overrides.items():
        if not isinstance(table, dict):
            continue
        if not key.endswith(".config_inject"):
            continue
        section = key[: -len(".config_inject")]
        out = _inject_section_keys(out, section, table)
    return out
