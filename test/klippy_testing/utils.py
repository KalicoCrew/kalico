import klippy.configfile


def configwrapper_to_dict(
    wrapper: klippy.configfile.ConfigWrapper,
) -> dict[str, dict[str, str]]:
    return {
        section: wrapper.fileconfig.items(section)
        for section in wrapper.fileconfig.sections()
    }
