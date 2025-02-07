# ------------------------------------------------------------------------------
#                                ALIASES
# ------------------------------------------------------------------------------

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
ROOT_DIR := $(MKFILE_DIR)
DISPLAY ?= :1
MAKE_MAP ?= True

# ------------------------------------------------------------------------------

COMPUTER_IMAGE = \
	robot

DOCKER_COMPOSE_FILES := \
	-f ./docker/docker-compose.yaml

BASE_IMAGES := $(COMPUTER_IMAGE)

# ------------------------------------------------------------------------------

RENDER_DISPLAY := $(DISPLAY)

BASE_PARAMETERS := \
	ROOT_DIR=$(ROOT_DIR) \
	RENDER_DISPLAY=$(DISPLAY) \ 
	MAKE_MAP=$(MAKE_MAP)

# ------------------------------------------------------------------------------

BUILD_COMMAND := $(BASE_PARAMETERS) docker compose $(DOCKER_COMPOSE_FILES) build

RUN_COMMAND := $(BASE_PARAMETERS) docker compose $(DOCKER_COMPOSE_FILES) up

# ------------------------------------------------------------------------------
#                              BUILDING COMMANDS
# ------------------------------------------------------------------------------

.PHONY: build-robot build-camera build-all

build-robot:
	@echo "Building robot"
	cd $(ROOT_DIR) && $(BUILD_COMMAND) $(COMPUTER_IMAGE)

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
	$(RUN_COMMAND) $(COMPUTER_IMAGE)

run-all-stack:
	@echo "Running all stack"
	cd $(ROOT_DIR) && \
	$(RUN_COMMAND) $(BASE_IMAGES)

run-pi:
	@echo "Running rpi4 script"
	chmod +x rpi_launch/rpi4_launch.sh
	cd $(ROOT_DIR)rpi_launch && ./rpi4_launch.sh

# ------------------------------------------------------------------------------
#                             AUXILIARY COMMANDS
# ------------------------------------------------------------------------------

.PHONY: prepare-for-visualization prepare-for-build

prepare-for-visualization:
	@echo "Preparing for visualization"
	DISPLAY=$(DISPLAY) xhost +local: && \
	DISPLAY=$(DISPLAY) xhost + && \
	export RCUTILS_COLORIZED_OUTPUT=1