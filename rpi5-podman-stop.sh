#!/usr/bin/env bash
set -euo pipefail

PODMAN_CONTAINER_NAME="${PODMAN_CONTAINER_NAME:-openmowernext}"

if podman container exists "${PODMAN_CONTAINER_NAME}"; then
  podman stop "${PODMAN_CONTAINER_NAME}"
  printf 'Stopped %s\n' "${PODMAN_CONTAINER_NAME}"
else
  printf 'Container %s does not exist\n' "${PODMAN_CONTAINER_NAME}"
fi
