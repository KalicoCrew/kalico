#!/usr/bin/env bash
#### Moonraker Timelapse component uninstaller
####
#### Copyright (C) 2021 Christoph Frei <fryakatkop@gmail.com>
#### Copyright (C) 2021 Stephan Wendel aka KwadFan <me@stephanwe.de>
####
#### This file may be distributed under the terms of the GNU GPLv3 license.
####

# shellcheck enable=require-variable-braces

## Error handling
set -Ee

## Debug Option
# set -x

### Check non-root
if [[ ${UID} = "0" ]]; then
    printf "\n\tYOU DONT NEED TO RUN INSTALLER AS ROOT!\n"
    printf "\tYou will be prompted for sudo password if needed!\nExiting...\n"
    exit 1
fi
### END

## Initialize global vars and arrays
DEPENDS_ON=( moonraker klipper )
MOONRAKER_TARGET_DIR="${HOME}/moonraker/moonraker/components"
SERVICES=()
### END

## Helper funcs
### Ask for proceding install (Step 2)
function continue_uninstall() {
    local reply
    while true; do
        read -erp "Would you like to proceed? [Y/n]: " -i "Y" reply
        case "${reply}" in
            [Yy]* )
                break
            ;;
            [Nn]* )
                abort_msg ### See Error messages
                exit 0
            ;;
            * )
                printf "\033[31mERROR: Please type Y or N !\033[0m"
            ;;
        esac
    done
}
### END

### Service related funcs
## Grab service names
function get_service_names() {
    for i in "${DEPENDS_ON[@]}"; do
        sudo systemctl list-units --full --all -t service --no-legend \
        | sed 's/^  //' | grep "^${i}*" | awk -F" " '{print $1}'
    done
}

## Build array from names
function set_service_name_array() {
    while read -r service ; do
        SERVICES+=("${service}")
    done < <(get_service_names)
}

## Stop related service (Step 4)
function stop_services() {
    local service
    ## Create services array
    set_service_name_array
    ## Dsiplay header message
    stop_service_header_msg
    ## Stop services
    for service in "${SERVICES[@]}"; do
        stop_service_msg "${service}"
        if sudo systemctl -q is-active "${service}"; then
            sleep 1
            sudo systemctl stop "${service}"
            service_stopped_msg
        else
            service_not_active_msg
        fi
    done
}

## Start related services (Step 9)
function start_services() {
    local service
    ## Dsiplay header message
    start_service_header_msg
    ## Stop services
    for service in "${SERVICES[@]}"; do
        start_service_msg "${service}"
        if ! sudo systemctl -q is-active "${service}"; then
            sleep 1
            sudo systemctl start "${service}"
            service_started_msg
        else
            service_start_failed_msg
        fi
    done
}
### END

### remove component (Step 4)
function remove_component() {
    if [ -d "${MOONRAKER_TARGET_DIR}" ]; then
        printf "Removing extension from moonraker ... "
        if rm -f "${MOONRAKER_TARGET_DIR}/timelapse.py" &> /dev/null; then
            printf "[\033[32mOK\033[0m]\n"
        else
            printf "[\033[31mFAILED\033[0m]\n"
        fi
    fi
}
### END

function remove_links() {
    local path
    path="$(find "${HOME}" -type l -name "timelapse.cfg" -printf "%P\n")"
    for i in ${path}; do
        printf "Removing timelapse.cfg from '%s'" "${i} ... "
        if rm -f "${HOME}/${i}"; then
                    printf "[\033[32mOK\033[0m]\n"
        else
            printf "[\033[31mFAILED\033[0m]\n"
        fi
    done
}

## Message helper funcs
### Welcome message (Step 1)
function welcome_msg() {
    printf "\n\033[31mAhoi!\033[0m\n"
    printf "moonraker-timelapse uninstall routine\n"
    printf "\n\tThis will take some time ...\n\tYou'll be prompted for sudo password if needed!\n"
    printf "\n\033[31m#################### WARNING #####################\033[0m\n"
    printf "Make sure you are \033[31mnot\033[0m printing during install!\n"
    printf "All related services will be stopped!\n"
    printf "\033[31m##################################################\033[0m\n\n"
}

### Service related msg (Step 3, Step 6)
function stop_service_header_msg() {
    printf "Stopping related service(s) ... \n"
}

function stop_service_msg() {
    printf "Stopping service '%s' ... " "${1}"
}

function service_stopped_msg() {
    printf "[\033[32mOK\033[0m]\n"
}

function service_not_active_msg() {
    printf "[\033[31mNOT ACTIVE\033[0m]\n"
}

function start_service_header_msg() {
    printf "Starting related service(s) ... \n"
}

function start_service_msg() {
    printf "Starting service '%s' ... " "${1}"
}

function service_started_msg() {
    printf "[\033[32mOK\033[0m]\n"
}

function service_start_failed_msg() {
    printf "[\033[31mFAILED\033[0m]\n"
}
### END


### Install finished message(s) (Step 7)
function finished_uninstall_msg() {
    printf "\nmoonraker-timelapse \033[32msuccessful\033[0m uninstalled ...\n"
    printf "\n\tPlease \033[31mDO NOT\033[0m forget to delete timelapse entrys from 'moonraker.conf'\n\n"
    printf "\033[34mHappy printing!\033[0m\n\n"
}

### END

### Error messages
function abort_msg() {
    printf "Install aborted by user ... \033[31mExiting!\033[0m\n"
}
### END

### MAIN
function main() {

# Step 1: Print welcome message
welcome_msg

# Step 2: Ask to proceed
continue_uninstall

# Step 3: Stop related services
stop_services

# Step 4: Remove component from $MOONRAKER_TARGET_DIR
remove_component

# Step 5: Remove links to timelapse.cfg
remove_links

# Step 6: Restart services
start_services

# Step 7: Print finish message
finished_uninstall_msg

}

## MAIN

main
exit 0

###### EOF ######
