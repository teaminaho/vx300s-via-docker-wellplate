#!/usr/bin/env bash
set -e
which yad

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

set +e
yad --borders=20 \
    --on-top \
    --sticky \
    --button="Start All":1 \
    --button="Stop All":2 \
    --button="Restart Arm Service":3 \
    --button="Restart GUI (Rviz)":4 \
    --title="Arm Service Control" \
    --text="Please choose action:"
rc=$?
[ $rc -eq 1 ] && ./startall.bash
[ $rc -eq 2 ] && ./stopall.bash
[ $rc -eq 3 ] && ./restart_arm_ctrl.bash
[ $rc -eq 4 ] && ./restart_rviz.bash
