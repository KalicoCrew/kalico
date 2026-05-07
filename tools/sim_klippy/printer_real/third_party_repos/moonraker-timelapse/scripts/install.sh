#!/usr/bin/env bash
#### Moonraker Timelapse component uninstaller
####
#### Copyright (C) 2021 till today Christoph Frei <fryakatkop@gmail.com>
#### Copyright (C) 2021 till today Stephan Wendel aka KwadFan <me@stephanwe.de>
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

## Find SRCDIR from the pathname of this script
SRC_DIR="$( cd "$(dirname "$(dirname "${BASH_SOURCE[0]}")")"/ && pwd )"
### END

## Initialize global vars and arrays
DATA_DIR=()
DEPENDS_ON=( moonraker klipper )
MOONRAKER_TARGET_DIR="${HOME}/moonraker/moonraker/components"
PKGLIST="wget"
SERVICES=()
### END

## Helper funcs
### Ask for proceding install (Step 2)
function continue_install() {
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

### Initial check func (Step 3)
function initial_check() {
    dep_check_msg
    for i in "${DEPENDS_ON[@]}"; do
        if [[ -d "${HOME}/${i}" ]]; then
            dep_found_msg "${i}"
        fi
        if [[ ! -d "${HOME}/${i}" ]]; then
            dep_not_found_msg "${i}"
        fi
    done
    printf "Installing core dependencies: '%s' ... \n" "${PKGLIST}"
    sudo apt-get update && sudo apt-get install --yes "${PKGLIST}"
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

### Determine Data sructure (Step 5)
## Get Instance names, also used for single instance installs
function get_instance_names() {
    local instances path
    instances="$(find "${HOME}" -maxdepth 1 -type d -name "*_data" -printf "%P\n")"
    while read -r path ; do
        DATA_DIR+=("${path}")
    done <<< "${instances}"
}

function determine_data_structure() {
    ## See if there is more then 'klipper_config'
    get_instance_names
    if [[ -d "${HOME}/klipper_config" ]] &&
    [[ "${#DATA_DIR[@]}" -eq 1 ]]; then
        DATA_DIR+=( klipper_config )
        printf "Old data structure found '%s'" "${DATA_DIR[0]}"
        return
    fi
    if [[ ! -d "${HOME}/klipper_config" ]] &&
    [[ "${#DATA_DIR[@]}" -eq 1 ]] &&
    [[ "${DATA_DIR[0]}" == "printer_data" ]] ; then
        printf "New data structure found '%s' (single instance)\n" "${DATA_DIR[0]}"
        return
    fi
    if [[ ! -d "${HOME}/klipper_config" ]] &&
    [[ "${#DATA_DIR[@]}" -gt 1 ]] ; then
        printf "New data structure found ... (Multi instance)\n"
        return
    fi
}
### END


### Link component (Step 6)
function link_component() {
    if [ -d "${MOONRAKER_TARGET_DIR}" ]; then
        printf "Linking extension to moonraker ... "
        if ln -sf "${SRC_DIR}/component/timelapse.py" \
        "${MOONRAKER_TARGET_DIR}/timelapse.py" &> /dev/null; then
            printf "[\033[32mOK\033[0m]\n"
        else
            printf "[\033[31mFAILED\033[0m]\n"
        fi
    fi
}

### Link timelapse.cfg (Step 7)
function link_macro_file() {
    local src p
    src="${SRC_DIR}/klipper_macro/timelapse.cfg"
    if [[ "${#DATA_DIR[@]}" -eq 1 ]] && [[ "${DATA_DIR[0]}" == "klipper_config" ]]; then
        link_to_msg "${DATA_DIR[0]}"
        if ln -sf "${src}" "${HOME}/${DATA_DIR[0]}/timelapse.cfg"; then
            link_to_ok_msg
            return
        else
            link_to_failed_msg
            return
        fi
    fi
    if [[ "${#DATA_DIR[@]}" -eq 1 ]] && [[ "${DATA_DIR[0]}" == "printer_data" ]]; then
        link_to_msg "${DATA_DIR[0]}"
        if ln -sf "${src}" "${HOME}/${DATA_DIR[0]}/config/timelapse.cfg"; then
            link_to_ok_msg
            return
        else
            link_to_failed_msg
            return
        fi
    fi
    if [[ "${#DATA_DIR[@]}" -gt 1 ]]; then
        for p in "${DATA_DIR[@]}"; do
            if [[ -d "${HOME}/${p}/config" ]]; then
                link_to_msg "${p}"
                if ln -sf "${src}" "${HOME}/${p}/config/timelapse.cfg"; then
                    link_to_ok_msg
                else
                    link_to_failed_msg
                fi
            fi
        done
    fi
}

### Check for ffmpeg (Step 8)
## Check if ffmpeg is installed, returns path if installed
function ffmpeg_installed() {
    local path
    path="$(command -v ffmpeg)"
    if [[ -n "${path}" ]]; then
        echo "${path}"
    fi
}

function check_ffmpeg() {
    if [[ -n "$(ffmpeg_installed)" ]]; then
        printf "Dependency 'ffmpeg' found in '%s'\n" "$(ffmpeg_installed)"
    else
        printf "Dependency 'ffmpeg' not found!\n"
        local reply
        while true; do
            read -erp "Would you like to install 'ffmpeg'? [Y/n]: " -i "Y" reply
            case "${reply}" in
                [Yy]* )
                    printf "Running 'apt-get update' ...\n"
                    sudo apt-get update
                    printf "Installing 'ffmpeg' ...\n"
                    sudo apt-get install --yes ffmpeg
                    break
                ;;
                [Nn]* )
                    printf "Installation of 'ffmpeg' skipped ...\n"
                    break
                ;;
                * )
                    printf "\033[31mERROR: Please type Y or N !\033[0m\n"
                ;;
            esac
        done
    fi
    return
}
### END

## Message helper funcs
### Welcome message (Step 1)
function welcome_msg() {
    printf "\n\033[31mAhoi!\033[0m\n"
    printf "moonraker-timelapse install routine\n"
    printf "\n\tThis will take some time ...\n\tYou'll be prompted for sudo password if needed!\n"
    printf "\n\033[31m#################### WARNING #####################\033[0m\n"
    printf "Make sure you are \033[31mnot\033[0m printing during install!\n"
    printf "All related services will be stopped!\n"
    printf "\033[31m##################################################\033[0m\n\n"
}

### Dependencie messages (Step 3)
function dep_check_msg() {
    printf "Check for dependencies to use moonraker-timelapse ...\n"
}

function dep_found_msg() {
    printf "Dependency '%s' found ... [\033[32mOK\033[0m]\n" "${1}"
}

function dep_not_found_msg() {
    printf "Dependency '%s' not found ... [\033[31mFAILED\033[0m]\n" "${1}"
    install_first_msg "${1}"
}
### END

### Service related msg (Step 4, Step 9)
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

### Link timelapse.cfg related messages (Step 7)
function link_to_msg() {
    printf "Linking timelapse.cfg to '%s' ... " "${1}"
}

function link_to_ok_msg() {
    printf "[\033[32mOK\033[0m]\n"
}

function link_to_failed_msg() {
    printf "[\033[31mFAILED\033[0m]\n"
}
### END

### Install finished message(s) (Step 10)
function finished_install_msg() {
    printf "\nmoonraker-timelapse \033[32msuccessful\033[0m installed ...\n"
    config_hint_header
    config_hint_block
    config_hint_footer
    printf "\033[34mHappy printing!\033[0m\n\n"
}

function config_hint_header() {
    printf "\nPlease add the following to your moonraker.conf:\n"
}

function config_hint_footer() {
    printf "\nFor further information please visit:\n"
    printf "https://github.com/mainsail-crew/moonraker-timelapse/blob/main/docs/configuration.md\n"
}

function config_hint_block(){
    if [[ "${DATA_DIR[0]}" == "klipper_config" ]]; then
        printf "\n\t- for Printer %s:\n" "${DATA_DIR[0]/_config/}"
        printf "\t[timelapse]\n\toutput_path: ~/%s/timelapse/\n" "${DATA_DIR[0]}"
        printf "\tframe_path: /tmp/timelapse\n"
        return
    fi
    for i in "${DATA_DIR[@]}"; do
        printf "\n\t- for Printer %s:\n" "${i/_data/}"
        printf "\t[timelapse]\n\toutput_path: ~/%s/timelapse/\n" "${i}"
        printf "\tframe_path: /tmp/timelapse/%s\n" "${i/_data/}"
    done
        if [[ "${#DATA_DIR[@]}" -ne "${#SERVICES[@]}" ]]; then
            printf "\nFolder structure and service count does not match!\n"
            printf "Please update only moonraker.conf of the installed printer\n\n"
        fi
    return
}
### END

### Error messages
function install_first_msg() {
    printf "Please install '%s' first! [\033[31mEXITING\033[0m]\n" "${1}"
    exit 1
}

function abort_msg() {
    printf "Install aborted by user ... \033[31mExiting!\033[0m\n"
}
### END

### MAIN
function main() {

# Step 1: Print welcome message
welcome_msg

# Step 2: Ask to proceed
continue_install

# Step 3: Initial checks for dependencies (klipper/moonraker)
initial_check

# Step 4: Stop related services
stop_services

# Step 5: Determine data structure
determine_data_structure

# Step 6: Link component to $MOONRAKER_TARGET_DIR
link_component

# Step 7: Link timelapse.cfg to $DATA_DIR
link_macro_file

# Step 8: Check for ffmpeg
check_ffmpeg

# Step 9: Restart services
start_services

# Step 10: Print finish message
finished_install_msg

}

## MAIN

main
exit 0

###### EOF ######
