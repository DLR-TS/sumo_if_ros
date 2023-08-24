SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

MAKEFLAGS += --no-print-directory

include sumo_if_ros.mk
include ${SUMO_IF_ROS_SUBMODULES_PATH}/ci_teststand/ci_teststand.mk

.PHONY: init_sumo_submodule
init_sumo_submodule:
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
  $(shell git submodule update --init --recursive --remote --depth 1 --jobs 4 --single-branch ${ROOT_DIR}/sumo/*)
else
	@echo "sumo submodule already initialized, skipping submodule init for sumo."
endif


.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?= 

SUMO_PROJECT:=sumo
SUMO_TAG:=v1_13_0
SUMO_IMAGE_NAME:=${SUMO_PROJECT}:${SUMO_TAG}

SUMO_DOCKER_ARCHIVE=/var/tmp/${SUMO_PROJECT}_${SUMO_TAG}.tar

DOCKER_REPOSITORY="andrewkoerner/adore"

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${SUMO_IF_ROS_PROJECT}) 
	$(eval TAG := ${SUMO_IF_ROS_TAG})


.PHONY: all
all: root_check docker_group_check build

.PHONY: save_docker_archive
save_docker_archive:
	@docker save -o "${SUMO_DOCKER_ARCHIVE}" "${SUMO_IMAGE_NAME}"

.PHONY: load_docker_archive
load_docker_archive:
	@docker load --input "${SUMO_DOCKER_ARCHIVE}" 2>/dev/null || true

.PHONY: load_sumo_image
load_sumo_image:
	@if [ ! -n "$$(docker images -q ${SUMO_IMAGE_NAME})" ]; then \
        make load_docker_archive;\
    fi
	@if [ ! -n "$$(docker images -q ${SUMO_IMAGE_NAME})" ]; then \
        make docker_pull;\
    fi

.PHONY: build_fast_sumo
build_fast_sumo: load_sumo_image
	@if [ -n "$$(docker images -q ${SUMO_IMAGE_NAME})" ]; then \
        echo "Docker image: ${SUMO_IMAGE_NAME} already build, skipping build."; \
    else \
        make load_sumo_image;\
        #make build_sumo;\
    fi
	cd "${ROOT_DIR}/sumo" && docker cp $$(docker create --rm ${SUMO_IMAGE_NAME}):/tmp/sumo/build build

.PHONY: clean_submodules
clean_submodules: clean_adore_if_ros_msg clean_coordinate_conversion clean_adore_v2x_sim

.PHONY: clean
clean: set_env ## Clean sumo_if_ros build artifacts
	$(eval MAKE_GADGETS_MAKEFILE_PATH := $(shell unset MAKE_GADGETS_MAKEFILE_PATH))
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
build: set_env start_apt_cacher_ng _build save_docker_archive get_cache_statistics ## Build sumo_if_ros 

.PHONY: _build
_build: set_env build_fast_sumo
	$(eval MAKE_GADGETS_MAKEFILE_PATH := $(shell unset MAKE_GADGETS_MAKEFILE_PATH))
	cd ${SUMO_IF_ROS_SUBMODULES_PATH}/adore_if_ros_msg && make build 
	cd ${SUMO_IF_ROS_SUBMODULES_PATH}/adore_v2x_sim && make build 
	cd ${SUMO_IF_ROS_SUBMODULES_PATH}/coordinate_conversion && make build 
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
build_sumo: init_sumo_submodule
	cd "${ROOT_DIR}/sumo" && rm -rf "build"
	cd "${ROOT_DIR}/sumo" && \
	docker build --network host \
                 --tag ${SUMO_IMAGE_NAME} .
	cd "${ROOT_DIR}/sumo" && docker cp $$(docker create --rm ${SUMO_IMAGE_NAME}):/tmp/sumo/build build

.PHONY: static_checks
static_checks: lizard cppcheck lint

.PHONY: docker_publish
docker_publish:
	docker tag "${SUMO_IMAGE_NAME}" "${DOCKER_REPOSITORY}:${SUMO_PROJECT}_${SUMO_TAG}"
	docker push ${DOCKER_REPOSITORY}:${SUMO_PROJECT}_${SUMO_TAG}

.PHONY: docker_pull
docker_pull:
	docker pull "${DOCKER_REPOSITORY}:${SUMO_PROJECT}_${SUMO_TAG}"
	docker tag "${DOCKER_REPOSITORY}:${SUMO_PROJECT}_${SUMO_TAG}" "${SUMO_IMAGE_NAME}"
	docker rmi "${DOCKER_REPOSITORY}:${SUMO_PROJECT}_${SUMO_TAG}"

.PHONY: test
test: ci_test
