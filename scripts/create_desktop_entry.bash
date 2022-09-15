#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
YAD_SCRIPTS_DIR="$(cd "${SCRIPT_DIR}/../yad-scripts" && pwd)"

TARGET="${HOME}/.local/share/applications/"

[ -d "${TARGET}" ] || exit 1

cat << EOS > "${TARGET}/arm_menu.desktop"
[Desktop Entry]
Name=Arm Control Menu
Exec="${YAD_SCRIPTS_DIR}/main.bash"
Comment=Robot Arm control menu
Terminal=false
Type=Application
EOS