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

# OR-Tools setup. Download the arch-correct v9.8 debian-11 tarball for both
# arm64 and amd64. The committed or-tools/{include,lib,bin} is intentionally
# overwritten here — the vendored amd64 copy in git is incomplete (missing
# absl/log/ headers and the bin/{protoc,scip,fzn-cp-sat} executables that
# protobuf-targets.cmake / scip-targets.cmake validate at configure time).
OR_TOOLS_DIR := $(CURDIR)/src/autonomy/exploration_planner/tare_planner/or-tools
OR_TOOLS_VERSION := v9.8.3296
OR_TOOLS_BASE := https://github.com/google/or-tools/releases/download/v9.8
OR_TOOLS_ARCHIVE ?= /tmp/or-tools.tar.gz
or-tools:
	@echo "=> Setting up OR-Tools..."
	@ARCH=$$(uname -m); \
	case "$$ARCH" in \
		aarch64) OR_TOOLS_TARBALL=or-tools_arm64_debian-11_cpp_$(OR_TOOLS_VERSION).tar.gz ;; \
		x86_64)  OR_TOOLS_TARBALL=or-tools_amd64_debian-11_cpp_$(OR_TOOLS_VERSION).tar.gz ;; \
		*) echo "=> Unsupported arch $$ARCH; OR-Tools setup skipped."; exit 0 ;; \
	esac; \
	echo "=> Detected $$ARCH, downloading $$OR_TOOLS_TARBALL..."; \
	wget -q --show-progress -O $(OR_TOOLS_ARCHIVE) $(OR_TOOLS_BASE)/$$OR_TOOLS_TARBALL && \
	echo "=> Extracting archive..." && \
	TMPDIR=$$(mktemp -d) && \
	tar -xzf $(OR_TOOLS_ARCHIVE) -C $$TMPDIR --strip-components=1 && \
	echo "=> Replacing include / lib / bin directories..." && \
	rm -rf $(OR_TOOLS_DIR)/include $(OR_TOOLS_DIR)/lib $(OR_TOOLS_DIR)/bin && \
	cp -a $$TMPDIR/include $(OR_TOOLS_DIR)/include && \
	cp -a $$TMPDIR/lib $(OR_TOOLS_DIR)/lib && \
	if [ -d $$TMPDIR/bin ]; then cp -a $$TMPDIR/bin $(OR_TOOLS_DIR)/bin; fi && \
	rm -rf $$TMPDIR $(OR_TOOLS_ARCHIVE) && \
	echo "=> OR-Tools $$ARCH setup complete."

# Docker commands. Host networking + privileged are required for ROS 2 DDS
# discovery and GPIO access on Jetson; --shm-size bumps /dev/shm so
# iceoryx/CycloneDDS shared-memory transports don't OOM.
DOCKER_RUN_FLAGS := -td --privileged --net=host --ipc=host --shm-size=2g \
    --name $(CONTAINER_NAME) --env-file $(ROBOT_ENV)

docker_stop:
	@echo "=> Stopping TypeGo SDK..."
	@docker rm -f $(CONTAINER_NAME) >/dev/null 2>&1 || true

docker_start: $(ROBOT_ENV) docker_stop
	@echo "=> Starting TypeGo SDK..."
	docker run $(DOCKER_RUN_FLAGS) $(IMAGE)

docker_open:
	@docker exec -it $(CONTAINER_NAME) bash

docker_remove: docker_stop
	@echo "=> Removing TypeGo SDK image..."
	@docker image rm -f $(IMAGE) >/dev/null 2>&1 || true

docker_build: $(ROBOT_ENV) docker_remove
	@echo "=> Building TypeGo SDK..."
	docker build --build-arg AUTONOMY_TYPE=$(AUTONOMY_TYPE) \
		-t $(IMAGE) -f $(DOCKERFILE) .
	@$(MAKE) docker_start

# Launch the full stack: render the env file, source ROS + workspace overlay,
# load the runtime env, then run typego_bringup. Pass extra launch args via
# ARGS="foo:=bar baz:=qux".
#
# SHELL is forced to bash because /opt/ros/humble/local_setup.bash uses
# ${BASH_SOURCE[0]} to locate itself; zsh doesn't set that, so sourcing it
# from a zsh recipe resolves paths against CWD and errors out.
.PHONY: launch
launch: SHELL := /bin/bash
launch: $(ROBOT_ENV)
	@if [ ! -f $(CURDIR)/install/local_setup.bash ]; then \
		echo "=> install/local_setup.bash missing — run 'make build' first."; \
		exit 1; \
	fi
	@{ \
		echo "=> Sourcing ROS + $(ROBOT_ENV)"; \
		source /opt/ros/humble/local_setup.bash; \
		source $(CURDIR)/install/local_setup.bash; \
		set -a; source $(ROBOT_ENV); set +a; \
		echo "=> ros2 launch typego_sdk typego_bringup.launch.py $(ARGS)"; \
		exec ros2 launch typego_sdk typego_bringup.launch.py $(ARGS); \
	}

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

# Save the current SLAM map. Auto-picks the right workflow from AUTONOMY_TYPE
# (base → slam_toolbox via docker; full → ARISE SLAM on host). All logic
# lives in scripts/save_map.py.
.PHONY: save_map
save_map: SHELL := /bin/bash
save_map: $(ROBOT_ENV)
	@if [ -z "$(FILE)" ]; then \
		echo "Error: FILE=<map-name> is required."; exit 1; \
	fi
	@set -a; source $(ROBOT_ENV); set +a; \
	 python3 $(CURDIR)/scripts/save_map.py $(FILE)

# Reset iox
iox_reset:
	sudo rm -rf /dev/shm/iceoryx*
	sudo rm -rf /dev/shm/iox_*
