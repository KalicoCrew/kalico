# Developing Plugins for Kalico

Kalico now supports loading external plugins packaged as a python distribution. This document is meant as a light guide for plugin authors to get started packaging their plugin.

Commonly, these plugins will simply be symlinked or copied into `klippy/extras/`. This has multiple downsides 
 however. Moonraker will note that Kalico is "dirty" (there are changes to files) and future changes to Kalico may conflict with a plugin.

`klippy/plugins/` was implemented to give a "safer" location for plugin authors to install to, however adoption has barely begun.

To make the whole system more user-friendly, and prevent some of these issues from arising, Kalico has implemented a Python-standard plugin loading system.


## A light overview of Python packaging

Python packages (often distributed as `my_package-v0.0.0.tar.gz` or `my_package-v0.0.0.whl`) are effectively just an installable zip file, adding code to a python environment.

Within these files you can define metadata that allows other packages to interact with yours in sepcific ways. 

Kalico's official plugin support takes advantage of the Python [Entry Points for Plugins](https://setuptools.pypa.io/en/latest/userguide/entry_point.html#entry-points-for-plugins) system, allowing for easy installation and discovery.

Once installed, a package with an entry-point under `kalico.plugins` can be discovered by Kalico and loaded at runtime (See `Printer.load_object()` in [`klippy/printer.py`](../klippy/printer.py)).

A full example plugin can be found at [github.com/KalicoCrew/plugin-example](https://github.com/KalicoCrew/plugin-example)


## Writing a new plugin from scratch

At it's core, a Kalico plugin is a python script that exposes functions for loading the plugin based on a configuration.

```python
from __future__ import annotations
import typing

if typing.TYPE_CHECKING:
    from klippy.configfile import ConfigWrapper


class MyPlugin:
    def __init__(self, config: ConfigWrapper):
        self.printer = config.get_printer()
        self.value = config.get("value")

    def get_status(self, eventtime: float | None = None) -> dict:
        "If defined, get_status is used to share information with other parts of Klippy"

        return {
            # In templates, `{printer.my_plugin.value}` will be available
            "value": self.value  
        }


def load_config(config: ConfigWrapper) -> MyPlugin:
    "Create an instance of MyPlugin when loaded as [my_plugin]"

    return MyPlugin(config)


# def load_config_prefix(config: ConfigWrapper) -> MyPlugin:
#     """
#     Create a named instance of MyPlugin, such as [my_plugin has_a_name]
#
#     config.get_name() would return the "my_plugin has_a_name". Often that name
#      will be read as `config.get_name().split(maxsplit=1)[-1]` leaving only "has_a_name"
#     """
#
#     return MyPlugin(config)
```

To package your plugin for use in Kalico, you need a build tool such as [`uv`](https://docs.astral.sh/uv/). 

Next to `my_plugin.py`, create a file named `pyproject.toml`.

```toml
[project]
name = "my_plugin"
version = "0.1"
description = "A good description of my plugin"
readme = "readme.md"
classifiers = [
    "Environment :: Plugins",
    "Framework :: Klippy",
]

[project.entry-points."kalico.plugins"]
my_plugin = "my_plugin"
```

You will also want to create `readme.md` with some example documentation about how to configure and use your plugin

At this point, you have the bare minimum to build and install your Kalico plugin.
`uv build` will package your plugin, creating `dist/my_plugin-0.1.tar.gz` and `dist/my_plugin-0.1-py3-none-any.whl`.

Kalico will discover the plugin on startup, expose the module's documentation to your frontend, and load the plugin when requested by the end users configuration.
