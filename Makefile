SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

MAKEFLAGS += --no-print-directory

include coordinate_conversion/make_gadgets/make_gadgets.mk
include coordinate_conversion/make_gadgets/docker/docker-tools.mk
include sumo_if_ros.mk

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?= 

REPO_DIRECTORY:="${ROOT_DIR}"

SUMO_PROJECT:="sumo"
SUMO_TAG:="v1_13_0"
sumo_TAG:="${SUMO_TAG}"
SUMO_IMAGE_NAME:="${SUMO_PROJECT}:${SUMO_TAG}"

PROJECT:=${SUMO_IF_ROS_PROJECT}
TAG:=${SUMO_IF_ROS_TAG}

include adore_if_ros_msg/adore_if_ros_msg.mk
include coordinate_conversion/coordinate_conversion.mk

.PHONY: all
all: root_check docker_group_check build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${SUMO_IF_ROS_PROJECT}) 
	$(eval TAG := ${SUMO_IF_ROS_TAG})

.PHONY: clean_submodules
clean_submodules: clean_adore_if_ros_msg clean_adore_v2x_sim clean_coordinate_conversion 

.PHONY: clean
clean: set_env ## Clean sumo_if_ros build artifacts
	make clean_submodules
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/sumo/build"
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	find . -name "**lint_report.log" -exec rm -rf {} \;
	docker rm $$(docker ps -a -q --filter "ancestor=${SUMO_IMAGE_NAME}") 2> /dev/null || true
	docker rmi $$(docker images -q ${SUMO_IMAGE_NAME}) 2> /dev/null || true
	docker rmi ${SUMO_IMAGE_NAME} --force 2> /dev/null
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) 2> /dev/null || true

.PHONY: build
build: set_env build_adore_if_ros_msg build_coordinate_conversion build_adore_v2x_sim build_sumo ## Build sumo_if_ros
	docker build --network host \
                 --tag ${PROJECT}:${TAG} \
                 --build-arg PROJECT=${PROJECT} \
                 --build-arg ADORE_IF_ROS_MSG_TAG=${ADORE_IF_ROS_MSG_TAG} \
                 --build-arg V2X_IF_ROS_MSG_TAG=${V2X_IF_ROS_MSG_TAG} \
                 --build-arg ADORE_V2X_SIM_TAG=${ADORE_V2X_SIM_TAG} \
                 --build-arg COORDINATE_CONVERSION_TAG=${COORDINATE_CONVERSION_TAG} \
                 --build-arg SUMO_TAG=${SUMO_TAG} .
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: build_sumo
build_sumo:
	cd "${ROOT_DIR}/sumo" && rm -rf "build"
	cd "${ROOT_DIR}/sumo" && \
	docker build --network host \
                 --tag ${SUMO_IMAGE_NAME} .
	cd "${ROOT_DIR}/sumo" && docker cp $$(docker create --rm ${SUMO_IMAGE_NAME}):/tmp/sumo/build build

.PHONY: static_checks
static_checks: lizard cppcheck lint
