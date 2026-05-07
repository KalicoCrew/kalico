# Installation

This document provides a guide on how to install Moonraker-timelapse component.
As this is a plugin to Moonraker, Moonraker and klipper needs to be installed
before hand.

## Installing the component

To install the Component you need to connect to your Raspberrypi via ssh and
execute following commands:

```
cd ~/
git clone https://github.com/mainsail-crew/moonraker-timelapse.git
cd ~/moonraker-timelapse
make install
```

This will clone the repository and execute the installer script.

> **NOTE:** The script assumes that Klipper is in your home directory under
> "klipper": `${HOME}/klipper` and "moonraker": `${HOME}\moonraker`.

> **NOTE:** Currently, there is a dummy systemd service installed
> to satisfy moonraker's update manager which also restarts Moonraker and Klipper.

> **NOTE:** Render functionality depends on ffmpeg in '/usr/bin/ffmpeg'.
> MainsailOS and fluiddPi have it installed there already. If you setup your OS
> manually you need to install ffmpeg manually too, to use the render function!
> To do so run: `sudo apt install ffmpeg`. If you have installed ffmpeg in a different
> directory you can specify the 'ffmpeg_binary_path' in moonraker.conf in the
> [timelapse] section

## Enable updating with moonraker update manager

This repo can be updated with the update manager of moonraker. To do so
add following to your 'moonraker.conf'

```
# moonraker.conf

[update_manager timelapse]
type: git_repo
primary_branch: main
path: ~/moonraker-timelapse
origin: https://github.com/mainsail-crew/moonraker-timelapse.git
managed_services: klipper moonraker
```

# Configuration

Please see [configuration.md](configuration.md) for details on how to
configure the timelapse component.
