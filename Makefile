REMOTE_HOST ?= omdev.local
REMOTE_USER ?= openmower
ROS_DISTRO ?= jazzy
ROS_LOG_DIR = log/
ROSBRIDGE_ADDRESS ?= 127.0.0.1
ROSBRIDGE_PORT ?= 9090
ROSBRIDGE_SERVICE ?= openmower-rosbridge.service
FOXGLOVE_ADDRESS ?= 0.0.0.0
FOXGLOVE_PORT ?= 8765
FOXGLOVE_SERVICE ?= openmower-foxglove.service
FOXGLOVE_USE_SIM_TIME ?= false
WEBOTS_STREAM ?= false
WEBOTS_PORT ?= 1234
DOCKER_IMAGE ?= openmowernext:local
DOCKER_BUILDER ?= openmower-builder
DOCKER_RPI_PLATFORM ?= linux/arm64/v8
DOCKER_BUILDX_CACHE ?= .buildx-cache
DOCKER_HARDWARE_TESTS_DIR ?= $(CURDIR)/hardware_tests
DOCKER_HARDWARE_TESTS_MOUNT ?= /opt/ws/hardware_tests
DOCKER_TEST_SCRIPTS_DIR ?= $(CURDIR)/scripts
DOCKER_TEST_SCRIPTS_MOUNT ?= $(DOCKER_HARDWARE_TESTS_MOUNT)/scripts
SYSTEMD_USER_DIR ?= $(HOME)/.config/systemd/user
SHELL := /bin/bash

all: custom-deps deps build

.PHONY: deps custom-deps build-libs build build-release docker-build docker-build-rpi docker-run docker-run-shell hardware-tests-init sim run dev run-foxglove foxglove foxglove-deps foxglove-service-install foxglove-service-enable foxglove-service-disable foxglove-service-restart foxglove-service-status foxglove-service-logs rsp remote-devices rosbridge rosbridge-deps rosbridge-service-install rosbridge-service-enable rosbridge-service-disable rosbridge-service-restart rosbridge-service-status rosbridge-service-logs

deps:
	rosdep install --from-paths . src/lib --ignore-src -i -y -r

custom-deps:
	sh utils/install-custom-deps.sh

build-libs:
	colcon build --base-paths "src/lib/*" --cmake-args -DBUILD_TESTING=OFF

build:
	colcon build --symlink-install

build-release:
	colcon build --base-paths "src/lib/*" --cmake-args -DCMAKE_BUILD_TYPE=Release
	source install/setup.bash && colcon build --packages-select open_mower_next --cmake-args -DCMAKE_BUILD_TYPE=Release

docker-build:
	docker compose build workspace

docker-build-rpi:
	mkdir -p "$(DOCKER_BUILDX_CACHE)"
	docker buildx build --builder "$(DOCKER_BUILDER)" --platform "$(DOCKER_RPI_PLATFORM)" --cache-from type=local,src="$(DOCKER_BUILDX_CACHE)" --cache-to type=local,dest="$(DOCKER_BUILDX_CACHE)-new",mode=max -t "$(DOCKER_IMAGE)" --load .
	rm -rf "$(DOCKER_BUILDX_CACHE)"
	mv "$(DOCKER_BUILDX_CACHE)-new" "$(DOCKER_BUILDX_CACHE)"

docker-run:
	docker compose up workspace

docker-run-shell: hardware-tests-init
	mkdir -p "$(DOCKER_HARDWARE_TESTS_DIR)"
	docker run --rm -it --privileged --network host --ipc host --security-opt seccomp=unconfined -v /dev:/dev -v "$(DOCKER_HARDWARE_TESTS_DIR):$(DOCKER_HARDWARE_TESTS_MOUNT)" -v "$(DOCKER_TEST_SCRIPTS_DIR):$(DOCKER_TEST_SCRIPTS_MOUNT):ro" "$(DOCKER_IMAGE)" bash

hardware-tests-init:
	mkdir -p "$(DOCKER_HARDWARE_TESTS_DIR)"
	if [ ! -f "$(DOCKER_HARDWARE_TESTS_DIR)/.env" ] && [ -f ".devcontainer/default.env" ]; then cp ".devcontainer/default.env" "$(DOCKER_HARDWARE_TESTS_DIR)/.env"; if grep -q '^export OM_MAP_PATH=' "$(DOCKER_HARDWARE_TESTS_DIR)/.env"; then sed -i 's|^export OM_MAP_PATH=.*|export OM_MAP_PATH=$(DOCKER_HARDWARE_TESTS_MOUNT)/map.geojson|' "$(DOCKER_HARDWARE_TESTS_DIR)/.env"; else printf '\nexport OM_MAP_PATH=$(DOCKER_HARDWARE_TESTS_MOUNT)/map.geojson\n' >> "$(DOCKER_HARDWARE_TESTS_DIR)/.env"; fi; fi
	if [ ! -f "$(DOCKER_HARDWARE_TESTS_DIR)/map.geojson" ] && [ -f ".devcontainer/home/map.geojson" ]; then cp ".devcontainer/home/map.geojson" "$(DOCKER_HARDWARE_TESTS_DIR)/map.geojson"; fi

sim:
	bash -lc 'if [ -z "$${WEBOTS_HOME}" ] && [ -d "$${HOME}/.ros/webotsR2025a/webots" ]; then export WEBOTS_HOME="$${HOME}/.ros/webotsR2025a/webots"; fi && if [[ "$${WEBOTS_HOME}" == /mnt/* ]]; then unset WEBOTS_OFFSCREEN; elif [ -z "$${DISPLAY}" ] && [ -z "$${WEBOTS_OFFSCREEN}" ]; then export WEBOTS_OFFSCREEN=1; fi && OM_MAP_PATH_OVERRIDE="$${OM_MAP_PATH:-}" && OM_DATUM_LAT_OVERRIDE="$${OM_DATUM_LAT:-}" && OM_DATUM_LONG_OVERRIDE="$${OM_DATUM_LONG:-}" && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash && set -a && source .devcontainer/default.env && set +a && if [ -n "$${OM_MAP_PATH_OVERRIDE}" ]; then export OM_MAP_PATH="$${OM_MAP_PATH_OVERRIDE}"; fi && if [ -n "$${OM_DATUM_LAT_OVERRIDE}" ]; then export OM_DATUM_LAT="$${OM_DATUM_LAT_OVERRIDE}"; fi && if [ -n "$${OM_DATUM_LONG_OVERRIDE}" ]; then export OM_DATUM_LONG="$${OM_DATUM_LONG_OVERRIDE}"; fi && ros2 launch open_mower_next sim.launch.py webots_stream:="$(WEBOTS_STREAM)" webots_port:="$(WEBOTS_PORT)"'

rosbridge:
	ROS_DISTRO="$(ROS_DISTRO)" ROSBRIDGE_ADDRESS="$(ROSBRIDGE_ADDRESS)" ROSBRIDGE_PORT="$(ROSBRIDGE_PORT)" bash utils/run-rosbridge.sh

rosbridge-deps:
	sudo apt update
	sudo apt install -y "ros-$(ROS_DISTRO)-rosbridge-server"

rosbridge-service-install:
	install -d "$(SYSTEMD_USER_DIR)"
	sed -e 's|@WORKSPACE@|$(CURDIR)|g' -e 's|@ROS_DISTRO@|$(ROS_DISTRO)|g' -e 's|@ROSBRIDGE_ADDRESS@|$(ROSBRIDGE_ADDRESS)|g' -e 's|@ROSBRIDGE_PORT@|$(ROSBRIDGE_PORT)|g' systemd/openmower-rosbridge.service.in > "$(SYSTEMD_USER_DIR)/$(ROSBRIDGE_SERVICE)"
	systemctl --user daemon-reload
	@printf 'Installed %s in %s\n' "$(ROSBRIDGE_SERVICE)" "$(SYSTEMD_USER_DIR)"

rosbridge-service-enable: rosbridge-service-install
	systemctl --user enable --now "$(ROSBRIDGE_SERVICE)"

rosbridge-service-disable:
	systemctl --user disable --now "$(ROSBRIDGE_SERVICE)"

rosbridge-service-restart:
	systemctl --user restart "$(ROSBRIDGE_SERVICE)"

rosbridge-service-status:
	systemctl --user status "$(ROSBRIDGE_SERVICE)"

rosbridge-service-logs:
	journalctl --user -u "$(ROSBRIDGE_SERVICE)" -f

foxglove:
	ROS_DISTRO="$(ROS_DISTRO)" FOXGLOVE_ADDRESS="$(FOXGLOVE_ADDRESS)" FOXGLOVE_PORT="$(FOXGLOVE_PORT)" FOXGLOVE_USE_SIM_TIME="$(FOXGLOVE_USE_SIM_TIME)" bash utils/run-foxglove.sh

foxglove-deps:
	sudo apt update
	sudo apt install -y "ros-$(ROS_DISTRO)-foxglove-bridge"

foxglove-service-install:
	install -d "$(SYSTEMD_USER_DIR)"
	sed -e 's|@WORKSPACE@|$(CURDIR)|g' -e 's|@ROS_DISTRO@|$(ROS_DISTRO)|g' -e 's|@FOXGLOVE_ADDRESS@|$(FOXGLOVE_ADDRESS)|g' -e 's|@FOXGLOVE_PORT@|$(FOXGLOVE_PORT)|g' -e 's|@FOXGLOVE_USE_SIM_TIME@|$(FOXGLOVE_USE_SIM_TIME)|g' systemd/openmower-foxglove.service.in > "$(SYSTEMD_USER_DIR)/$(FOXGLOVE_SERVICE)"
	systemctl --user daemon-reload
	@printf 'Installed %s in %s\n' "$(FOXGLOVE_SERVICE)" "$(SYSTEMD_USER_DIR)"

foxglove-service-enable: foxglove-service-install
	systemctl --user enable --now "$(FOXGLOVE_SERVICE)"

foxglove-service-disable:
	systemctl --user disable --now "$(FOXGLOVE_SERVICE)"

foxglove-service-restart:
	systemctl --user restart "$(FOXGLOVE_SERVICE)"

foxglove-service-status:
	systemctl --user status "$(FOXGLOVE_SERVICE)"

foxglove-service-logs:
	journalctl --user -u "$(FOXGLOVE_SERVICE)" -f

run:
	ros2 launch launch/openmower.launch.py

dev:
	cd .devcontainer && docker-compose up -d

run-foxglove:
	$(MAKE) foxglove

rsp:
	ros2 launch launch/rsp.launch.py

remote-devices:
	bash .devcontainer/scripts/remote_devices.sh $(REMOTE_HOST) $(REMOTE_USER)
