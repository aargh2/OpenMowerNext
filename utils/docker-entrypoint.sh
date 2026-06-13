#!/bin/bash
set -e

preserve_existing_env() {
  if [ -v OM_DATUM_LAT ]; then
    _existing_om_datum_lat="${OM_DATUM_LAT}"
    _had_om_datum_lat=1
  fi
  if [ -v OM_DATUM_LONG ]; then
    _existing_om_datum_long="${OM_DATUM_LONG}"
    _had_om_datum_long=1
  fi
  if [ -v OM_MAP_PATH ]; then
    _existing_om_map_path="${OM_MAP_PATH}"
    _had_om_map_path=1
  fi
}

restore_existing_env() {
  if [ "${_had_om_datum_lat:-0}" = "1" ]; then
    export OM_DATUM_LAT="${_existing_om_datum_lat}"
  fi
  if [ "${_had_om_datum_long:-0}" = "1" ]; then
    export OM_DATUM_LONG="${_existing_om_datum_long}"
  fi
  if [ "${_had_om_map_path:-0}" = "1" ]; then
    export OM_MAP_PATH="${_existing_om_map_path}"
  fi
}

preserve_existing_env
if [ -f "${WORKSPACE}/.devcontainer/default.env" ]; then
  source "${WORKSPACE}/.devcontainer/default.env"
fi
if [ -f "${WORKSPACE}/.devcontainer/override/.env" ]; then
  source "${WORKSPACE}/.devcontainer/override/.env"
fi
restore_existing_env

if [ -z "${OM_DATUM_LAT}" ]; then
  export OM_DATUM_LAT=30.0

  echo "OM_DATUM_LAT not set, using default value: ${OM_DATUM_LAT}"
fi

if [ -z "${OM_DATUM_LONG}" ]; then
  export OM_DATUM_LONG=0.5

  echo "OM_DATUM_LONG not set, using default value: ${OM_DATUM_LONG}"
fi

if [ -z "${OM_MAP_PATH}" ]; then
  export OM_MAP_PATH=${WORKSPACE}/map.json

  echo "OM_MAP_PATH not set, using default value: ${OM_MAP_PATH}"
fi

case "${OM_MAP_PATH}" in
  /*) ;;
  *) export OM_MAP_PATH="${WORKSPACE}/${OM_MAP_PATH}" ;;
esac

if [ ! -f "${OM_MAP_PATH}" ]; then
  mkdir -p "$(dirname "${OM_MAP_PATH}")"
  echo '{"type": "FeatureCollection", "features": []}' > "${OM_MAP_PATH}"
  echo "Created empty map file at ${OM_MAP_PATH}"
fi

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
  source ${WORKSPACE}/install/local_setup.bash
fi

# Execute the command passed to the container
exec "$@"
