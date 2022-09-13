#!/usr/bin/env bash
set -e
which yad
which docker

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
COMPOSE_DIR="$(cd "${SCRIPT_DIR}/../" && pwd)"

cd "${COMPOSE_DIR}" && docker compose restart -t 1 arm_ctrl_rviz
