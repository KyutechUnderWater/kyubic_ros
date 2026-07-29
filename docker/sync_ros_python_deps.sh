#!/bin/bash
# ROS 2 console_scripts use /usr/bin/python3, not the project .venv.
# Copy selected packages from .venv into the user site-packages so nodes like
# mavlink_driver can import them without PyPI access at runtime.

set -euo pipefail

VENV_SITE="${HOME}/kyubic_ros/.venv/lib/python3.12/site-packages"
USER_SITE="${HOME}/.local/lib/python3.12/site-packages"
PACKAGES=(pymavlink fastcrc lxml)

if [[ ! -d "${VENV_SITE}" ]]; then
	echo "sync_ros_python_deps: venv site-packages not found at ${VENV_SITE}" >&2
	exit 0
fi

mkdir -p "${USER_SITE}"

for pkg in "${PACKAGES[@]}"; do
	if [[ -d "${VENV_SITE}/${pkg}" ]]; then
		cp -a "${VENV_SITE}/${pkg}" "${USER_SITE}/"
	fi
	for dist_info in "${VENV_SITE}/${pkg}-"*.dist-info; do
		if [[ -d "${dist_info}" ]]; then
			cp -a "${dist_info}" "${USER_SITE}/"
		fi
	done
done
