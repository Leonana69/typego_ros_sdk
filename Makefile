DETECTED_SHELL := $(or $(shell echo $$SHELL),/bin/bash)
SHELL := $(DETECTED_SHELL)

DOCKERFILE = ./docker/Dockerfile
IMAGE = typego-sdk:0.1
CONTAINER_NAME = typego-sdk

# --- Single-source robot config ---
# robot.yaml is the source of truth. The Makefile renders a shell-sourceable
# env file at /tmp/typego-runtime.env via `typego-config env`, then reads
# build-time knobs (AUTONOMY_TYPE, ROBOT_ID) from it via `-include`.
# docker/.env is no longer read; delete it once you've verified the migration.
ROBOT_YAML := $(CURDIR)/src/typego_config/config/robot.yaml
ROBOT_ENV  := /tmp/typego-runtime.env
TYPEGO_CONFIG_CLI := PYTHONPATH=$(CURDIR)/src/typego_config python3 -m typego_config.cli

# Render the env file whenever robot.yaml or the bootstrap CLI is newer.
$(ROBOT_ENV): $(ROBOT_YAML) $(CURDIR)/src/typego_config/typego_config/env.py
	@$(TYPEGO_CONFIG_CLI) env --config $(ROBOT_YAML) > $@.tmp && mv $@.tmp $@

.PHONY: robot_env
robot_env: $(ROBOT_ENV)
	@:

# Import the rendered variables. The file must exist on first `make`, so
# commands that need it depend on `$(ROBOT_ENV)` explicitly; the -include
# below merely folds values into $(AUTONOMY_TYPE) etc. once available.
-include $(ROBOT_ENV)
AUTONOMY_TYPE := $(strip $(AUTONOMY_TYPE))

.PHONY: docker_stop docker_start docker_remove docker_open docker_build build setup or-tools

# Console scripts for ament_python packages that need their shebang patched
# to the active python3 interpreter (colcon hardcodes whichever python3 was on
# PATH at build time, which is wrong when building inside a conda env for a
# user who runs from system python — or vice-versa).
PYTHON_ENTRYPOINTS := \
	./install/typego_web_gateway/lib/typego_web_gateway/gateway_node \
	./install/typego_config/lib/typego_config/config_service_node \
	./install/typego_config/lib/typego_config/typego-config

# Build the project
build: $(ROBOT_ENV)
	@if [ "$(AUTONOMY_TYPE)" = "base" ]; then \
		echo "=> AUTONOMY_TYPE=base, excluding autonomy packages..."; \
		AUTONOMY_PACKAGES=$$(find src/autonomy -name "package.xml" \
			-exec grep -h "<name>" {} \; \
			| sed 's/.*<name>\(.*\)<\/name>.*/\1/' \
			| tr '\n' ' '); \
		echo "=> Skipping packages: $$AUTONOMY_PACKAGES"; \
		if [ -n "$$AUTONOMY_PACKAGES" ]; then \
			colcon build --packages-skip $$(echo $$AUTONOMY_PACKAGES); \
		else \
			echo "Warning: No autonomy packages found to skip"; \
			colcon build; \
		fi; \
	else \
		echo "=> AUTONOMY_TYPE=$(AUTONOMY_TYPE:-unset), building all packages..."; \
		colcon build; \
	fi
	@PYTHON=$$(which python3 2>/dev/null || which python 2>/dev/null || echo "/usr/bin/python3"); \
	echo "=> Patching ament_python entry-point shebangs to $$PYTHON"; \
	for ep in $(PYTHON_ENTRYPOINTS); do \
		if [ -f "$$ep" ]; then \
			sed -i '1s|^#!.*|#!'"$$PYTHON"'|' "$$ep"; \
			dos2unix "$$ep" 2>/dev/null || sed -i 's/\r$$//' "$$ep"; \
			chmod +x "$$ep"; \
		else \
			echo "   warning: $$ep not found, skipping shebang patch"; \
		fi; \
	done

# Validate robot.yaml without building (fast pre-flight check).
.PHONY: config_validate
config_validate:
	@$(TYPEGO_CONFIG_CLI) validate --config $(ROBOT_YAML)

# Setup dependencies for full autonomy
# 1. SLAM dependencies (Sophus + gtsam)
# 2. OR-Tools
setup: $(ROBOT_ENV)
	@echo "=> Setting up TypeGo SDK..."
	@if [ "$(AUTONOMY_TYPE)" = "full" ]; then \
		echo "=> Building SLAM dependencies (Sophus + gtsam)..."; \
		cd $(CURDIR)/src/autonomy/slam/dependency/Sophus && \
		rm -rf build && mkdir build && cd build && \
		cmake .. -DBUILD_TESTS=OFF && \
		make && sudo make install; \
		# cd $(CURDIR)/src/autonomy/slam/dependency/gtsam && \
		# rm -rf build && mkdir build && cd build && \
		# cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
		# make -j6 && sudo make install && \
		# sudo /sbin/ldconfig -v; \
		$(MAKE) -C $(CURDIR) or-tools; \
	else \
		echo "=> AUTONOMY_TYPE=$(or $(AUTONOMY_TYPE),unset), skipping SLAM dependency build."; \
	fi

# OR-Tools setup, download a different version for arm64
OR_TOOLS_DIR := $(CURDIR)/src/autonomy/exploration_planner/tare_planner/or-tools
OR_TOOLS_URL := https://github.com/google/or-tools/releases/download/v9.8/or-tools_arm64_debian-11_cpp_v9.8.3296.tar.gz
OR_TOOLS_ARCHIVE := /tmp/or-tools_arm64.tar.gz
or-tools:
	@echo "=> Setting up OR-Tools..."
	@ARCH=$$(uname -m); \
	if [ "$$ARCH" = "aarch64" ]; then \
		echo "=> Detected aarch64, downloading arm64 OR-Tools..."; \
		wget -q --show-progress -O $(OR_TOOLS_ARCHIVE) $(OR_TOOLS_URL) && \
		echo "=> Extracting archive..." && \
		TMPDIR=$$(mktemp -d) && \
		tar -xzf $(OR_TOOLS_ARCHIVE) -C $$TMPDIR --strip-components=1 && \
		echo "=> Replacing include and lib directories..." && \
		rm -rf $(OR_TOOLS_DIR)/include $(OR_TOOLS_DIR)/lib && \
		cp -r $$TMPDIR/include $(OR_TOOLS_DIR)/include && \
		cp -r $$TMPDIR/lib $(OR_TOOLS_DIR)/lib && \
		rm -rf $$TMPDIR $(OR_TOOLS_ARCHIVE) && \
		echo "=> OR-Tools arm64 setup complete."; \
	else \
		echo "=> Detected $$ARCH, using default OR-Tools (amd64)."; \
	fi

# Docker commands
docker_stop:
	@echo "=> Stopping TypeGo SDK..."
	@-docker stop -t 0 $(CONTAINER_NAME) > /dev/null 2>&1
	@-docker rm -f $(CONTAINER_NAME) > /dev/null 2>&1

docker_start: $(ROBOT_ENV)
	@make docker_stop
	@echo "=> Starting TypeGo SDK..."
	docker run -td --privileged --net=host --ipc=host \
    	--name="$(CONTAINER_NAME)" \
		--shm-size=2g \
		--env-file $(ROBOT_ENV) \
		$(IMAGE)

docker_remove:
	@echo "=> Removing TypeGo SDK..."
	@-docker image rm -f $(IMAGE)  > /dev/null 2>&1
	@-docker rm -f $(CONTAINER_NAME) > /dev/null 2>&1

docker_open:
	@echo "=> Opening bash in TypeGo SDK..."
	@docker exec -it $(CONTAINER_NAME) bash

docker_build: $(ROBOT_ENV)
	@echo "=> Building TypeGo SDK..."
	@make docker_stop
	@make docker_remove
	@echo -n "=>"
	docker build \
		--build-arg AUTONOMY_TYPE=$(AUTONOMY_TYPE) \
		-t $(IMAGE) -f $(DOCKERFILE) .
	@echo -n "=>"
	@make docker_start

# Run rviz
rviz: $(ROBOT_ENV)
	@{ \
		echo "→ Loading $(ROBOT_ENV)"; \
		set -a; source $(ROBOT_ENV); set +a; \
		if [ -n "$${ROBOT_ID}" ]; then \
			ros2 run rviz2 rviz2 $(if $(filter full,$(AUTONOMY_TYPE)),-d $(CURDIR)/src/autonomy/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz) --ros-args -r /tf:=/robot$${ROBOT_ID}/tf -r /tf_static:=/robot$${ROBOT_ID}/tf_static -r /goal_pose:=/robot$${ROBOT_ID}/goal_pose; \
		else \
			ros2 run rviz2 rviz2 $(if $(filter full,$(AUTONOMY_TYPE)),-d $(CURDIR)/src/autonomy/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz); \
		fi; \
	}

# Save map for base autonomy
save_map_base_autonomy:
	@echo "=> Saving map..."
	@{ \
        if [ -z "$(FILE)" ]; then \
            echo "Error: FILE variable is not set. Please set FILE to the desired filename."; \
            exit 1; \
        fi; \
        echo '$(FILE)'; \
    }
	mkdir -p $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)
	ros2 run typego_sdk get_position_node $(CURDIR)/src/typego_sdk/resource/Map-$(FILE) --ros-args -r /tf:=/robot$(ROBOT_ID)/tf

	docker exec $(CONTAINER_NAME) \
		bash -c "source /opt/ros/humble/setup.bash && \
		/opt/ros/humble/bin/ros2 service call $(if $(ROBOT_ID),/robot$(ROBOT_ID),)/slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph \"{filename: '/workspace/$(FILE)'}\""
	docker cp $(CONTAINER_NAME):/workspace/$(FILE).posegraph $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/$(FILE).posegraph
	docker cp $(CONTAINER_NAME):/workspace/$(FILE).data $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/$(FILE).data
	docker cp $(CONTAINER_NAME):/workspace/install/typego_sdk/share/typego_sdk/resource/Map-empty_map/waypoints.csv $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/waypoints.csv

# Save map for full autonomy
save_map_full_autonomy:
	@echo "=> Saving map..."
	@{ \
        if [ -z "$(FILE)" ]; then \
            echo "Error: FILE variable is not set. Please set FILE to the desired filename."; \
            exit 1; \
        fi; \
        echo '$(FILE)'; \
    }
	mkdir -p $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)
	ros2 run typego_sdk get_position_node $(CURDIR)/src/typego_sdk/resource/Map-$(FILE) --ros-args -r /tf:=$(if $(ROBOT_ID),/robot$(ROBOT_ID),)/tf -r /tf_static:=$(if $(ROBOT_ID),/robot$(ROBOT_ID),)/tf_static

	ros2 service call /save_slam_map arise_slam_mid360_msgs/srv/SaveSlamMap "{file_path: '$(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/$(FILE)'}"
	ros2 service call /save_explored_areas visualization_tools/srv/SaveExploredAreas "{file_path: '$(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/$(FILE)'}"
	cp $(CURDIR)/install/typego_sdk/share/typego_sdk/resource/Map-empty_map/waypoints.csv $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/waypoints.csv
	@echo "=> Syncing map to install directory..."
	mkdir -p $(CURDIR)/install/typego_sdk/share/typego_sdk/resource/Map-$(FILE)
	cp -r $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/ $(CURDIR)/install/typego_sdk/share/typego_sdk/resource/Map-$(FILE)/

# Reset iox
iox_reset:
	sudo rm -rf /dev/shm/iceoryx*
	sudo rm -rf /dev/shm/iox_*
