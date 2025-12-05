#!/bin/bash

# Set alias
set_alias_command() {
    local command="$1"

    # extract alias command
    re='(alias) (\S+)='
    if [[ ${command} =~ ${re} ]]; then
        local alias_command=${BASH_REMATCH[2]}
        if ! (alias -p "${alias_command}" >&/dev/null); then
            echo $command >>~/.bash_aliases
            echo "Registered ${command}"
        else
            echo "Already registered ${alias_command}"
        fi
    else
        echo "Don't find alias command"
    fi

}

set_alias_command "alias kyubic_ping_ping_sensor_esp32='ping 192.168.9.5'"
set_alias_command "alias kyubic_ping_dvl='ping 192.168.9.10'"
set_alias_command "alias kyubic_ping_gnss='ping 192.168.9.20'"
set_alias_command "alias kyubic_ping_main='ping 192.168.9.100'"
set_alias_command "alias kyubic_ping_main_kvm='ping 192.168.9.105'"
set_alias_command "alias kyubic_ping_img='ping 192.168.9.110'"
set_alias_command "alias kyubic_ping_img_kvm='ping 192.168.9.115'"
set_alias_command "alias kyubic_ping_mic='ping 192.168.9.120'"

set_alias_command "alias kyubic_ssh_main='ssh kyubic@192.168.9.100'"
set_alias_command "alias kyubic_ssh_img='ssh kyubic@192.168.9.110'"
set_alias_command "alias kyubic_ssh_mic='ssh kyubic@192.168.9.120'"

set_alias_command "alias kyubic_shutdown_main='ssh kyubic@192.168.9.100 sudo -S shutdown -h now'"
set_alias_command "alias kyubic_shutdown_img='ssh kyubic@192.168.9.110 sudo -S shutdown -h now'"
set_alias_command "alias kyubic_shutdown_mic='ssh kyubic@192.168.9.120 sudo -S shutdown -h now'"

source ~/.bash_aliases
