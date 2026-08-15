import types

from scripts import buildcommands


def test_handle_kconfig_generate_code_reads_file(tmp_path):
    defconfig = tmp_path / "defconfig"
    defconfig.write_text("CONFIG_MACH_STM32=y\nCONFIG_MACH_STM32F042=y\n")
    handler = buildcommands.HandleKConfig()
    options = types.SimpleNamespace(kconfig=str(defconfig))

    code = handler.generate_code(options)

    assert code == ""
    assert handler.defconfig == "CONFIG_MACH_STM32=y\nCONFIG_MACH_STM32F042=y\n"


def test_handle_kconfig_update_data_dictionary():
    handler = buildcommands.HandleKConfig()
    handler.defconfig = "CONFIG_MACH_STM32=y\n"
    data = {}

    handler.update_data_dictionary(data)

    assert data == {"kconfig": "CONFIG_MACH_STM32=y\n"}


def test_handle_kconfig_accepts_empty_defconfig(tmp_path):
    # A board matching Kconfig defaults exactly produces an empty
    # defconfig; that's valid, not a build failure.
    defconfig = tmp_path / "defconfig"
    defconfig.write_text("")
    handler = buildcommands.HandleKConfig()
    options = types.SimpleNamespace(kconfig=str(defconfig))

    code = handler.generate_code(options)

    assert code == ""
    data = {}
    handler.update_data_dictionary(data)
    assert data == {"kconfig": ""}


def test_handle_kconfig_registered_before_handle_identify():
    kconfig_indices = [
        i
        for i, h in enumerate(buildcommands.Handlers)
        if isinstance(h, buildcommands.HandleKConfig)
    ]
    identify_indices = [
        i
        for i, h in enumerate(buildcommands.Handlers)
        if isinstance(h, buildcommands.HandleIdentify)
    ]
    assert kconfig_indices and identify_indices
    assert kconfig_indices[0] < identify_indices[0]
