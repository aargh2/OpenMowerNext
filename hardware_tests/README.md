# Hardware test runtime files

This directory is mounted into the runtime container at `/opt/ws/hardware_tests`.

Put local hardware-test configuration in `.env` and the editable map in `map.geojson`.
Both files are ignored by git so caster credentials, local datum values, and recorded maps
do not get committed accidentally.

Example `.env`:

```bash
export OM_DATUM_LAT=49.0
export OM_DATUM_LONG=15.0
export OM_MAP_PATH=/opt/ws/hardware_tests/map.geojson
export OM_NTRIP_ENABLED=true
export OM_NTRIP_HOSTNAME=example.local
export OM_NTRIP_PORT=2101
export OM_NTRIP_ENDPOINT=MOUNT
export OM_NTRIP_AUTHENTICATE=true
export OM_NTRIP_USER=user
export OM_NTRIP_PASSWORD=password
```
