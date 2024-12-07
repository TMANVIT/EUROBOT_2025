# ------------------------------------------------------------------------------
#                                ALIASES
# ------------------------------------------------------------------------------

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
ROOT_DIR := $(MKFILE_DIR)
DISPLAY ?= :1

# ------------------------------------------------------------------------------

ROBOT_IMAGE := \
	robot
CAMERA_IMAGE := \
	camera

DOCKER_COMPOSE_FILES := \
	-f docker-compose.yaml

BASE_IMAGES := $(ROBOT_IMAGE) $(CAMERA_IMAGE)

# ------------------------------------------------------------------------------

RENDER_DISPLAY := $(DISPLAY)

BASE_PARAMETERS := \
	ROOT_DIR=$(ROOT_DIR) \
	RENDER_DISPLAY=$(DISPLAY)

# ------------------------------------------------------------------------------

BUILD_COMMAND := docker compose $(DOCKER_COMPOSE_FILES) build

RUN_COMMAND := docker compose $(DOCKER_COMPOSE_FILES) up

# ------------------------------------------------------------------------------
#                              BUILDING COMMANDS
# ------------------------------------------------------------------------------

.PHONY: build-robot build-camera build-all

build-robot:
	@echo "Building robot"
	cd $(ROOT_DIR) && $(BASE_PARAMETERS) $(BUILD_COMMAND) $(ROBOT_IMAGE)

build-camera:
	@echo "Building camera"
	cd $(ROOT_DIR) && $(BASE_PARAMETERS) $(BUILD_COMMAND) $(CAMERA_IMAGE)

build-all:
	@echo "Building all"
	cd $(ROOT_DIR) && $(BASE_PARAMETERS) $(BUILD_COMMAND) $(BASE_IMAGES)

# ------------------------------------------------------------------------------
#                              RUNNING COMMANDS
# ------------------------------------------------------------------------------

.PHONY: run-robot-stack run-camera-stack run-all-stack

run-robot-stack: 
	@echo "Running robot stack"
	cd $(ROOT_DIR) && \
	$(RUN_COMMAND) $(ROBOT_IMAGE)

run-camera-stack: prepare-for-visualization
	@echo "Running camera stack"
	cd $(ROOT_DIR) && \
	$(RUN_COMMAND) $(CAMERA_IMAGE)

run-all-stack:
	@echo "Running all stack"
	cd $(ROOT_DIR) && \
	$(RUN_COMMAND) $(BASE_IMAGES)

# ------------------------------------------------------------------------------
#                             AUXILIARY COMMANDS
# ------------------------------------------------------------------------------

.PHONY: prepare-for-visualization prepare-for-build

prepare-for-visualization:
	@echo "Preparing for visualization"
	DISPLAY=$(DISPLAY) xhost +local: && \
	DISPLAY=$(DISPLAY) xhost + && \
	export RCUTILS_COLORIZED_OUTPUT=1

prepare-for-build:
	@echo "Initializing git submodules"
	git submodule init
