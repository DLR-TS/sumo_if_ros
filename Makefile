SHELL:=/bin/bash

.DEFAULT_GOAL := all

SUMO_IMAGE_NAME="sumo:v1_13_0"
SUMO_IF_ROS_IMAGE_NAME="sumo_if_ros:latest"
PROJECT="sumo_if_ros"
VERSION="latest"
TAG="${PROJECT}:${VERSION}"

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

MAKEFLAGS += --no-print-directory
.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: help
help:
	@awk 'BEGIN {FS = ":.*##"; printf "Usage: make \033[36m<target>\033[0m\n"} /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

.PHONY: all
all: build

.PHONY: clean
clean:
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/sumo/build"
	cd adore_if_ros_msg && make clean
	cd adore_v2x_sim && make clean
	docker rm $$(docker ps -a -q --filter "ancestor=${SUMO_IMAGE_NAME}") 2> /dev/null || true
	docker rmi $$(docker images -q ${SUMO_IMAGE_NAME}) 2> /dev/null || true
	docker rmi ${SUMO_IMAGE_NAME} --force 2> /dev/null
	docker rm $$(docker ps -a -q --filter "ancestor=${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${TAG}) 2> /dev/null || true
	docker rmi ${TAG} --force 2> /dev/null

.PHONY: build
build: clean build_adore_if_ros_msg  build_adore_v2x_sim build_sumo build_sumo_if_ros

.PHONY: build_adore_if_ros_msg
build_adore_if_ros_msg:
	cd "${ROOT_DIR}/adore_if_ros_msg" && \
	make

.PHONY: build_adore_v2x_sim
build_adore_v2x_sim:
	cd "${ROOT_DIR}/adore_v2x_sim" && \
	make

.PHONY: build_sumo_if_ros
build_sumo_if_ros:
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker build --network host \
                 --tag $(shell echo ${TAG} | tr A-Z a-z) \
                 --build-arg PROJECT=${PROJECT} .
	docker cp $$(docker create --rm ${TAG}):/tmp/${PROJECT}/build ${ROOT_DIR}/${PROJECT}

.PHONY: build_sumo
build_sumo:
	cd "${ROOT_DIR}/sumo" && rm -rf "build"
	cd "${ROOT_DIR}/sumo" && \
	docker build --network host \
                 --tag ${SUMO_IMAGE_NAME} .
	cd "${ROOT_DIR}/sumo" && docker cp $$(docker create --rm ${SUMO_IMAGE_NAME}):/tmp/sumo/build build

.PHONY: lint
lint:
	cd cpplint_docker && \
        make lint CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)

.PHONY: lintfix 
lintfix:
	cd cpplint_docker && \
        make lintfix CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)

.PHONY: lintfix_simulate
lintfix_simulate:
	cd cpplint_docker && \
        make lintfix_simulate CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)


.PHONY: cppcheck 
cppcheck:
	cd cppcheck_docker && \
        make cppcheck CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)

.PHONY: lizard 
lizard:
	cd lizard_docker && \
        make lizard CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)
