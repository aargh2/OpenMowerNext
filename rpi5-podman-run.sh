#!/usr/bin/env bash
set -euo pipefail

PODMAN_IMAGE="${PODMAN_IMAGE:-openmowernext:rpi5}"
PODMAN_CONTAINER_NAME="${PODMAN_CONTAINER_NAME:-openmowernext}"
PODMAN_HARDWARE_TESTS_DIR="${PODMAN_HARDWARE_TESTS_DIR:-${PWD}/hardware_tests}"
PODMAN_TEST_SCRIPTS_DIR="${PODMAN_TEST_SCRIPTS_DIR:-${PWD}/scripts}"
PODMAN_HARDWARE_TESTS_MOUNT="${PODMAN_HARDWARE_TESTS_MOUNT:-/opt/ws/hardware_tests}"
PODMAN_TEST_SCRIPTS_MOUNT="${PODMAN_TEST_SCRIPTS_MOUNT:-${PODMAN_HARDWARE_TESTS_MOUNT}/scripts}"

mkdir -p "${PODMAN_HARDWARE_TESTS_DIR}"

if [ ! -f "${PODMAN_HARDWARE_TESTS_DIR}/.env" ] && [ -f ".devcontainer/default.env" ]; then
  cp ".devcontainer/default.env" "${PODMAN_HARDWARE_TESTS_DIR}/.env"
  if grep -q '^export OM_MAP_PATH=' "${PODMAN_HARDWARE_TESTS_DIR}/.env"; then
    sed -i "s|^export OM_MAP_PATH=.*|export OM_MAP_PATH=${PODMAN_HARDWARE_TESTS_MOUNT}/map.geojson|" "${PODMAN_HARDWARE_TESTS_DIR}/.env"
  else
    printf '\nexport OM_MAP_PATH=%s/map.geojson\n' "${PODMAN_HARDWARE_TESTS_MOUNT}" >>"${PODMAN_HARDWARE_TESTS_DIR}/.env"
  fi
fi

if [ ! -f "${PODMAN_HARDWARE_TESTS_DIR}/map.geojson" ] && [ -f ".devcontainer/home/map.geojson" ]; then
  cp ".devcontainer/home/map.geojson" "${PODMAN_HARDWARE_TESTS_DIR}/map.geojson"
fi

sudo podman rm -f "${PODMAN_CONTAINER_NAME}" >/dev/null 2>&1 || true

sudo podman run -d \
  --name "${PODMAN_CONTAINER_NAME}" \
  --user root \
  --privileged \
  --network host \
  --ipc host \
  --security-opt seccomp=unconfined \
  --device "${OM_HOST_MAINBOARD_SERIAL_PORT:-/dev/ttyAMA0}:/dev/ttyAMA0" \
  --device "${OM_HOST_GPS_SERIAL_PORT:-/dev/ttyAMA1}:/dev/ttyAMA1" \
  --device "${OM_HOST_VESC_RIGHT_SERIAL_PORT:-/dev/ttyAMA2}:/dev/ttyAMA2" \
  --device "${OM_HOST_VESC_MOWER_SERIAL_PORT:-/dev/ttyAMA3}:/dev/ttyAMA3" \
  --device "${OM_HOST_VESC_LEFT_SERIAL_PORT:-/dev/ttyAMA4}:/dev/ttyAMA4" \
  -v "${PODMAN_HARDWARE_TESTS_DIR}:${PODMAN_HARDWARE_TESTS_MOUNT}" \
  -v "${PODMAN_TEST_SCRIPTS_DIR}:${PODMAN_TEST_SCRIPTS_MOUNT}:ro" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  -e ROSBRIDGE_ADDRESS="${ROSBRIDGE_ADDRESS:-0.0.0.0}" \
  -e ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}" \
  "${PODMAN_IMAGE}"

printf 'Started %s from %s\n' "${PODMAN_CONTAINER_NAME}" "${PODMAN_IMAGE}"
