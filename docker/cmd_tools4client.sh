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

set_alias_command "alias kyubic_dvl_ping='ping 192.168.9.10'"
set_alias_command "alias kyubic_main_ping='ping 192.168.9.100'"
set_alias_command "alias kyubic_img_ping='ping 192.168.9.110'"
set_alias_command "alias kyubic_mic_ping='ping 192.168.9.120'"
set_alias_command "alias kyubic_gnss_ping='ping 192.168.9.130'"
set_alias_command "alias kyubic_main_shutdown='ssh kyubic_main sudo -S shutdown -h now'"
set_alias_command "alias kyubic_img_shutdown='ssh kyubic_img sudo -S shutdown -h now'"
set_alias_command "alias kyubic_mic_shutdown='ssh kyubic_mic sudo -S shutdown -h now'"

source ~/.bash_aliases
