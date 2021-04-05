.DEFAULT_GOAL := help

VERSION := 0.1
CONTAINER := cast_simulations_$(VERSION)
SOURCE_MOUNT:=/root/catkin_ws/src/

JOYSTICK_DEVICE := /dev/input/js0
JOYSTICK_FLAGS := --device $(JOYSTICK_DEVICE):$(JOYSTICK_DEVICE)

USE_GPU := false
GPU_FLAGS := --gpus=all

FORWARD_X := false
XAUTH := /tmp/.docker.xauth
DISPLAY_FORWARDING_FLAGS := --env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$(XAUTH)" \
	--volume="$(XAUTH):$(XAUTH)" \
	--volume="/tmp:/tmp:rw"

DOCKER_RUN_BASE = docker run --rm -it\
	--volume $(shell pwd):$(SOURCE_MOUNT) \
	--volume /dev/input:/dev/input \
  --network host \
	--privileged \
	$(JOYSTICK_FLAGS) \
	$(if $(FORWARD_X),$(DISPLAY_FORWARDING_FLAGS)) \
	--volume /dev:/dev
# /dev mounting for realsense and joystick

ENTRY_COMMAND = /bin/bash

DOCKER_RUN = $(DOCKER_RUN_BASE) \
  $(CONTAINER) $(ENTRY_COMMAND)

DOCKER_RUN_GPU = $(DOCKER_RUN_BASE) \
	$(if $(USE_GPU),$(GPU_FLAGS)) \
  $(CONTAINER) $(ENTRY_COMMAND)

.PHONY: help
help: ## Display this help message
	@grep -hE '^\S+:.*##' $(MAKEFILE_LIST) | sed -e 's/:[[:blank:]]*\(##\)[[:blank:]]*/\1/' | column -s '##' -t

.PHONY: up
up: ## Launch the container
	@$(DOCKER_RUN)

.PHONY: up-display
up-display: FORWARD_X := true
up-display: ## Laucnh the container and forward the X display
	@xhost +local:root
	@xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $(XAUTH) nmerge -
	@$(DOCKER_RUN)
	@xhost -local:root

.PHONY: up-display-gpu
up-display-gpu: FORWARD_X := true
up-display-gpu: USE_GPU := true
up-display-gpu: ## Laucnh the container and forward the X display with GPU support. Required for those with CUDA installed
	@xhost +local:root
	@xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $(XAUTH) nmerge -
	@$(DOCKER_RUN_GPU)
	@xhost -local:root

.PHONY: build
build: ## Build the container image
	@echo "Starting Docker build with the following command"
	@DOCKER_BUILDKIT=1 docker build -t $(CONTAINER) .
