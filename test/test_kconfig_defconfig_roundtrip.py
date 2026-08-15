import os
import pathlib
import subprocess
import sys
import types

import pytest

ROOT = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "lib" / "kconfiglib"))

import kconfiglib  # noqa: E402

KCONFIG = str(ROOT / "src" / "Kconfig")
CONFIGS = sorted((ROOT / "test" / "configs").glob("*.config")) + sorted(
    (ROOT / "board_configs").glob("*.config")
)


@pytest.fixture(scope="module", autouse=True)
def _repo_root_cwd():
    # src/Kconfig sources the generated, gitignored src/extras/Kconfig,
    # and kconfiglib resolves "source" paths relative to cwd, not the
    # Kconfig file's location. Regenerate src/extras/ and run from the
    # repo root so this doesn't depend on `make` having already run.
    previous = os.getcwd()
    os.chdir(ROOT)
    subprocess.run(
        ["bash", str(ROOT / "scripts" / "find-firmware-extras.sh")],
        cwd=ROOT,
        check=True,
    )
    try:
        yield
    finally:
        os.chdir(previous)


@pytest.mark.parametrize("config_path", CONFIGS, ids=lambda p: p.stem)
def test_defconfig_roundtrip_reproduces_expanded_config(config_path, tmp_path):
    # Loading a minimized defconfig into a fresh Kconfig instance must
    # expand back to the config that produced it. Compare canonical
    # write_config() output, not raw defconfig text.
    kconf = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
    kconf.load_config(str(config_path), replace=True)

    expanded_path = tmp_path / "expanded.config"
    kconf.write_config(str(expanded_path), save_old=False)
    expanded_before = expanded_path.read_text()

    defconfig_path = tmp_path / "defconfig"
    kconf.write_min_config(str(defconfig_path))

    kconf2 = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
    kconf2.load_config(str(defconfig_path), replace=True)
    # kconfiglib silently drops assignments to symbols the tree doesn't
    # define — assert there are none for a same-tree round trip.
    assert kconf2.missing_syms == []

    replayed_path = tmp_path / "replayed.config"
    kconf2.write_config(str(replayed_path), save_old=False)
    assert replayed_path.read_text() == expanded_before


def test_handlekconfig_accepts_a_real_all_defaults_board_config(tmp_path):
    # atmega2560.config matches the Kconfig tree's defaults exactly, so
    # write_min_config() produces a zero-byte defconfig for it.
    # HandleKConfig must accept that, not treat it as a failure.
    from scripts import buildcommands

    kconf = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
    kconf.load_config(
        str(ROOT / "test" / "configs" / "atmega2560.config"), replace=True
    )
    defconfig_path = tmp_path / "defconfig"
    kconf.write_min_config(str(defconfig_path))

    # Verify the empty case actually occurs rather than assuming it.
    assert defconfig_path.read_text() == ""

    handler = buildcommands.HandleKConfig()
    options = types.SimpleNamespace(kconfig=str(defconfig_path))
    handler.generate_code(options)  # must not raise

    data = {}
    handler.update_data_dictionary(data)
    assert data == {"kconfig": defconfig_path.read_text()}
