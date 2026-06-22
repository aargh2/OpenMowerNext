#!/usr/bin/env bash
set -euo pipefail

DOCKER_REGISTRY="${DOCKER_REGISTRY:-192.168.1.106:6000}"
PODMAN_IMAGE="${PODMAN_IMAGE:-openmowernext:rpi5}"
PODMAN_REGISTRY_IMAGE="${PODMAN_REGISTRY_IMAGE:-${DOCKER_REGISTRY}/${PODMAN_IMAGE}}"
PODMAN_TLS_VERIFY="${PODMAN_TLS_VERIFY:-false}"

sudo podman pull --tls-verify="${PODMAN_TLS_VERIFY}" "${PODMAN_REGISTRY_IMAGE}"
sudo podman tag "${PODMAN_REGISTRY_IMAGE}" "${PODMAN_IMAGE}"

printf 'Pulled %s and tagged it as %s\n' "${PODMAN_REGISTRY_IMAGE}" "${PODMAN_IMAGE}"
