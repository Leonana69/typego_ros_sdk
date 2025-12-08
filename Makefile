SHELL := /bin/zsh

# --- Default environment setup ---
ENV_FILE := ./docker/.env
-include $(ENV_FILE)

SETUP_FILE := ./install/setup.zsh  # can also be setup.bash if preferred

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
	@echo "=> Stopping go2-sdk..."
	@-docker stop -t 0 go2-sdk > /dev/null 2>&1
	@-docker rm -f go2-sdk > /dev/null 2>&1

docker_start:
	@make docker_stop
	@echo "=> Starting go2-sdk..."
	docker run -td --privileged --net=host --ipc=host \
    	--name="go2-sdk" \
		--shm-size=2g \
		--env-file $(ENV_FILE) \
		go2-sdk:0.1

docker_remove:
	@echo "=> Removing go2-sdk..."
	@-docker image rm -f go2-sdk:0.1  > /dev/null 2>&1
	@-docker rm -f go2-sdk > /dev/null 2>&1

docker_open:
	@echo "=> Opening bash in go2-sdk..."
	@docker exec -it go2-sdk bash

docker_build:
	@echo "=> Building go2-sdk..."
	@make docker_stop
	@make docker_remove
	@echo -n "=>"
	docker build --build-arg ROBOT_TYPE=$(ROBOT_TYPE) -t go2-sdk:0.1 -f ./docker/Dockerfile .
	@echo -n "=>"
	@make docker_start

save_map:
	@echo "=> Saving map..."
	@{ \
        if [ -z "$(FILE)" ]; then \
            echo "Error: FILE variable is not set. Please set FILE to the desired filename."; \
            exit 1; \
        fi; \
        echo '$(FILE)'; \
    }
	docker exec go2-sdk \
		bash -c "source /opt/ros/humble/setup.bash && \
		/opt/ros/humble/bin/ros2 service call $(if $(ROBOT_ID),/robot$(ROBOT_ID),)/slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph \"{filename: '/workspace/$(FILE)'}\""
	
	mkdir -p $(CURDIR)/src/go2_sdk/resource/Map-$(FILE)
	docker cp go2-sdk:/workspace/$(FILE).posegraph $(CURDIR)/src/go2_sdk/resource/Map-$(FILE)/$(FILE).posegraph
	docker cp go2-sdk:/workspace/$(FILE).data $(CURDIR)/src/go2_sdk/resource/Map-$(FILE)/$(FILE).data
	# docker cp go2-sdk:/workspace/$(FILE).pgm $(CURDIR)/src/go2_sdk/resource/Map-$(FILE)/waypoints.csv