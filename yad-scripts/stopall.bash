#!/usr/bin/env bash
set -e
which yad
which docker

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
COMPOSE_DIR="$(cd "${SCRIPT_DIR}/../" && pwd)"

cd "${COMPOSE_DIR}" &&
    yad --borders=20 \
        --on-top \
        --sticky \
        --image="gtk-dialog-warning" \
        --button=gtk-ok:0 \
        --button=gtk-cancel:1 \
        --title="SAFETY WARNING" \
        --text="Make sure the arm is on rest position for safety.\nContinue?" &&
    docker compose down -t 2
