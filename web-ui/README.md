# OpenMower Next Web UI

Static dashboard for a mower running OpenMower Next.

## Run

Start rosbridge on the mower:

```sh
make rosbridge
```

Build and run the dedicated web UI image:

```sh
make docker-build-web-ui
make docker-run-web-ui
```

The container serves the dashboard on `WEB_UI_PORT`, default `8080`.

To build an ARM64 image on a non-RPi machine and save it for transfer:

```sh
make docker-build-web-ui-rpi-save
```

This writes `openmowernext-web-ui-rpi.tar` by default. Override `WEB_UI_RPI_IMAGE_TAR`
to choose a different export path.

For local development, serve this directory directly:

```sh
cd web-ui
python3 -m http.server 8080
```

Then open `http://<mower-host>:8080` and connect to `ws://<mower-host>:9090`.

## ROS interfaces

The app reads:

- `/power` (`sensor_msgs/msg/BatteryState`)
- `/power/charger_present` (`std_msgs/msg/Bool`)
- `/power/charge_voltage` (`std_msgs/msg/Float32`)
- `/hardware/emergency` (`std_msgs/msg/Bool`)
- `/hardware/rain` (`std_msgs/msg/Bool`)
- `/hardware/ui_event` (`open_mower_next/msg/UiButtonEvent`)
- `/gps/fix` (`sensor_msgs/msg/NavSatFix`)
- `/gps/odom` (`nav_msgs/msg/Odometry`)

The app writes:

- `/hardware/ui_event` (`open_mower_next/msg/UiButtonEvent`)
- `/hardware/set_emergency` (`std_srvs/srv/SetBool`)
- `/hardware/clear_emergency` (`std_srvs/srv/Trigger`)
