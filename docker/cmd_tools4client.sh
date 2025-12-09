#!/bin/bash

# Set alias
set_alias_command() {
    local command="$1"
    local re='^alias[[:space:]]+([^=]+)='

    if [[ ${command} =~ ${re} ]]; then
        local alias_name="${BASH_REMATCH[1]}"

        # 空文字チェック
        if [[ -z "$alias_name" ]]; then
            echo "Error: Could not extract alias name."
            return 1
        fi

        if grep -Fq "alias ${alias_name}=" ~/.bash_aliases; then
            echo "Already registered in ~/.bash_aliases: ${alias_name}"
            return 0
        else
            echo "$command" >>~/.bash_aliases
            echo "Registered: ${command}"
            eval "$command"
        fi
    else
        echo "Error: '${command}' is not a valid alias format"
    fi
}

set_alias_command "alias kyubic_ping_ping_sensor_esp32='ping 192.168.9.5'"
set_alias_command "alias kyubic_ping_dvl='ping 192.168.9.10'"
set_alias_command "alias kyubic_ping_gnss='ping 192.168.9.20'"
set_alias_command "alias kyubic_ping_main='ping 192.168.9.100'"
set_alias_command "alias kyubic_ping_main_kvm='ping 192.168.9.105'"
set_alias_command "alias kyubic_ping_jetson='ping 192.168.9.110'"
set_alias_command "alias kyubic_ping_jetson_kvm='ping 192.168.9.115'"
set_alias_command "alias kyubic_ping_rpi5='ping 192.168.9.120'"

set_alias_command "alias kyubic_ssh_main='ssh kyubic_main'"
set_alias_command "alias kyubic_ssh_jetson='ssh kyubic_jetson'"
set_alias_command "alias kyubic_ssh_rpi5='ssh kyubic_rpi5'"

set_alias_command "alias kyubic_shutdown_main='ssh -t kyubic_main sudo shutdown -h now'"
set_alias_command "alias kyubic_shutdown_jetson='ssh -t kyubic_jetson sudo shutdown -h now'"
set_alias_command "alias kyubic_shutdown_rpi5='ssh -t kyubic_rpi5 sudo shutdown -h now'"
