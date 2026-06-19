#!/usr/bin/env bash
set -euo pipefail

DOCKER_REGISTRY="${DOCKER_REGISTRY:-192.168.1.106:6000}"
PODMAN_IMAGE="${PODMAN_IMAGE:-openmowernext:rpi5}"
PODMAN_REGISTRY_IMAGE="${PODMAN_REGISTRY_IMAGE:-${DOCKER_REGISTRY}/${PODMAN_IMAGE}}"
PODMAN_CONTAINER_NAME="${PODMAN_CONTAINER_NAME:-openmowernext}"

if podman container exists "${PODMAN_CONTAINER_NAME}"; then
  podman rm -f "${PODMAN_CONTAINER_NAME}"
  printf 'Removed container %s\n' "${PODMAN_CONTAINER_NAME}"
fi

for image in "${PODMAN_IMAGE}" "${PODMAN_REGISTRY_IMAGE}"; do
  if podman image exists "${image}"; then
    podman rmi "${image}"
    printf 'Removed image %s\n' "${image}"
  fi
done
