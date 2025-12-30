DETECTED_SHELL := $(or $(shell echo $$SHELL),/bin/bash)
SHELL := $(DETECTED_SHELL)

DOCKERFILE = ./docker/Dockerfile
IMAGE = typego-sdk:0.1
CONTAINER_NAME = typego-sdk

# --- Default environment setup ---
ENV_FILE := ./docker/.env
-include $(ENV_FILE)

.PHONY: docker_stop docker_start docker_remove docker_open docker_build build

build:
	colcon build
	@{ \
		if [ -f ./install/typego/lib/typego/webui ]; then \
			sed -i '1s|^#!.*|#!'"$$(which python)"'|' ./install/typego/lib/typego/webui; \
		else \
			echo "Warning: ./install/typego/lib/typego/webui not found. Skipping shebang update."; \
		fi \
	}

docker_stop:
	@echo "=> Stopping TypeGo SDK..."
	@-docker stop -t 0 $(CONTAINER_NAME) > /dev/null 2>&1
	@-docker rm -f $(CONTAINER_NAME) > /dev/null 2>&1

docker_start:
	@make docker_stop
	@echo "=> Starting TypeGo SDK..."
	docker run -td --privileged --net=host --ipc=host \
    	--name="$(CONTAINER_NAME)" \
		--shm-size=2g \
		--env-file $(ENV_FILE) \
		$(IMAGE)

docker_remove:
	@echo "=> Removing TypeGo SDK..."
	@-docker image rm -f $(IMAGE)  > /dev/null 2>&1
	@-docker rm -f $(CONTAINER_NAME) > /dev/null 2>&1

docker_open:
	@echo "=> Opening bash in TypeGo SDK..."
	@docker exec -it $(CONTAINER_NAME) bash

docker_build:
	@echo "=> Building TypeGo SDK..."
	@make docker_stop
	@make docker_remove
	@echo -n "=>"
	docker build -t $(IMAGE) -f $(DOCKERFILE) .
	@echo -n "=>"
	@make docker_start

rviz:
	@{ \
		echo "→ Loading $(ENV_FILE)"; \
		set -a; source $(ENV_FILE); set +a; \
		ros2 run rviz2 rviz2 --ros-args -r /tf:=/robot$${ROBOT_ID}/tf -r /tf_static:=/robot$${ROBOT_ID}/tf_static -r /goal_pose:=/robot$${ROBOT_ID}/goal_pose; \
	}

save_map:
	@echo "=> Saving map..."
	@{ \
        if [ -z "$(FILE)" ]; then \
            echo "Error: FILE variable is not set. Please set FILE to the desired filename."; \
            exit 1; \
        fi; \
        echo '$(FILE)'; \
    }
	docker exec $(CONTAINER_NAME) \
		bash -c "source /opt/ros/humble/setup.bash && \
		/opt/ros/humble/bin/ros2 service call $(if $(ROBOT_ID),/robot$(ROBOT_ID),)/slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph \"{filename: '/workspace/$(FILE)'}\""
	
	mkdir -p $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)
	docker cp $(CONTAINER_NAME):/workspace/$(FILE).posegraph $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/$(FILE).posegraph
	docker cp $(CONTAINER_NAME):/workspace/$(FILE).data $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/$(FILE).data
	docker cp $(CONTAINER_NAME):/workspace/install/typego_sdk/share/typego_sdk/resource/Map-empty_map/waypoints.csv $(CURDIR)/src/typego_sdk/resource/Map-$(FILE)/waypoints.csv
	ros2 run typego_sdk get_position_node ./src/typego_sdk/resource/Map-$(FILE) --ros-args -r /tf:=/robot$(ROBOT_ID)/tf	