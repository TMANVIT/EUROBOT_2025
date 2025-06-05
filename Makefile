# ------------------------------------------------------------------------------
#                                ALIASES
# ------------------------------------------------------------------------------

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
ROOT_DIR := $(MKFILE_DIR)
DISPLAY ?= :1
MAKE_MAP ?= false
RMW_IMPLEMENTATION ?= rmw_cyclonedds_cpp
CYCLONEDDS_URI ?= /ros2_ws/cyclonedds.xml
ROS_DOMAIN_ID := 0
TEAM := 1 ## 0 - Yellow; 1 - Blue

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
	MAKE_MAP=$(MAKE_MAP) \
	RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION) \
	CYCLONEDDS_URI=$(CYCLONEDDS_URI) \
	ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
	TEAM=$(TEAM)

# ------------------------------------------------------------------------------

BUILD_COMMAND := $(BASE_PARAMETERS) docker compose $(DOCKER_COMPOSE_FILES) build

RUN_COMMAND := docker compose $(DOCKER_COMPOSE_FILES) up

# ------------------------------------------------------------------------------
#                              BUILDING COMMANDS
# ------------------------------------------------------------------------------

.PHONY: build-robot build-all

build-robot:
	@echo "Building robot"
	cd $(ROOT_DIR) && $(BUILD_COMMAND) $(COMPUTER_IMAGE)

build-all:
	@echo "Building all"
	cd $(ROOT_DIR) && $(BASE_PARAMETERS) $(BUILD_COMMAND) $(BASE_IMAGES)

# ------------------------------------------------------------------------------
#                              RUNNING COMMANDS
# ------------------------------------------------------------------------------

.PHONY: run-yellow run-blue run-all-stack  run-pi

run-yellow:
	@echo "Running robot stack (Yellow team)"
	cd $(ROOT_DIR) && $(BASE_PARAMETERS) TEAM=0 $(RUN_COMMAND) $(COMPUTER_IMAGE)

run-blue:
	@echo "Running robot stack (Blue team)"
	cd $(ROOT_DIR) && $(BASE_PARAMETERS) TEAM=1 $(RUN_COMMAND) $(COMPUTER_IMAGE)

run-all-stack:
	@echo "Running all stack"
	cd $(ROOT_DIR) && \
	$(BASE_PARAMETERS) $(RUN_COMMAND) $(BASE_IMAGES)

run-pi:
	@echo "Running rpi4 script"
	chmod +x rpi_launch/rpi4_launch.sh
	cd $(ROOT_DIR)rpi_launch && ./rpi4_launch.sh

# ------------------------------------------------------------------------------
#                             AUXILIARY COMMANDS
# ------------------------------------------------------------------------------

.PHONY: prepare-for-visualization

prepare-for-visualization:
	@echo "Preparing for visualization"
	DISPLAY=$(DISPLAY) xhost +local: && \
	DISPLAY=$(DISPLAY) xhost + && \
    export RCUTILS_COLORIZED_OUTPUT=1