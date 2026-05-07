#!/bin/bash
repo=motors-sync
repo_path="$(cd "$(dirname "$0")" && pwd)"

# Exit if root
if [ "$(id -u)" = "0" ]; then
    echo "Script must run from non-root !!!"
    exit 1
fi

klipper_path="$HOME/klipper"
module_name=motors_sync.py
module_path=${klipper_path}/klippy/extras/

# Linking
ln -sf "$repo_path/$module_name" "${module_path}${module_name}"

# Add to .git/info/exclude if not already present
exclude_file="${klipper_path}/.git/info/exclude"
exclude_entry="klippy/extras/${module_name}"
if ! grep -qxF "$exclude_entry" "$exclude_file"; then
    echo "$exclude_entry" >> "$exclude_file"
fi

blk_path=~/printer_data/config/moonraker.conf
# Include update block in moonraker.conf
if [ -f "$blk_path" ]; then
    if ! grep -q "^\[update_manager $repo\]$" "$blk_path"; then
        read -p " Do you want to install updater? (y/n): " answer
        if [ "$answer" != "${answer#[Yy]}" ]; then
          sudo service moonraker stop
          sed -i "\$a \ " "$blk_path"
          sed -i "\$a [update_manager $repo]" "$blk_path"
          sed -i "\$a type: git_repo" "$blk_path"
          sed -i "\$a path: $repo_path" "$blk_path"
          sed -i "\$a origin: https://github.com/MRX8024/$repo.git" "$blk_path"
          sed -i "\$a primary_branch: main" "$blk_path"
          sed -i "\$a managed_services: klipper" "$blk_path"
		  sed -i "\$a virtualenv: $HOME/klippy-env/" "$blk_path"
          sed -i "\$a requirements: wiki/requirements.txt" "$blk_path"
          sed -i "\$a system_dependencies: wiki/packages.json" "$blk_path"
          # echo "Including [update_manager] to $blk_path successfully complete"
          sudo service moonraker start
        else
          echo "Installing updater aborted"
        fi
    else
        echo "Including [update_manager] aborted, [update_manager] already exists in $blk_path"
    fi
fi

sudo apt update
sudo apt install libatlas-base-dev libopenblas-dev
~/klippy-env/bin/pip install -r "$repo_path/"wiki/requirements.txt
