include make_gadgets/Makefile
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

.PHONY: all
all: root_check docker_group_check build

.PHONY: clean
clean: ## Cleans the build artifacts 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/sumo/build"
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	find . -name "**lint_report.log" -exec rm -rf {} \;
	cd adore_v2x_sim && make clean
	cd coordinate_conversion && make clean
	cd v2x_if_ros_msg && make clean
	cd adore_if_ros_msg && make clean
	docker rm $$(docker ps -a -q --filter "ancestor=${SUMO_IMAGE_NAME}") 2> /dev/null || true
	docker rmi $$(docker images -q ${SUMO_IMAGE_NAME}) 2> /dev/null || true
	docker rmi ${SUMO_IMAGE_NAME} --force 2> /dev/null
	docker rm $$(docker ps -a -q --filter "ancestor=${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${TAG}) 2> /dev/null || true
	docker rmi ${TAG} --force 2> /dev/null

.PHONY: build
build: clean build_adore_if_ros_msg  build_v2x_if_ros_msg build_adore_v2x_sim build_sumo build_sumo_if_ros ## Build sumo_if_ros

.PHONY: build_adore_if_ros_msg
build_adore_if_ros_msg:
	cd "${ROOT_DIR}/adore_if_ros_msg" && \
	make

.PHONY: build_v2x_if_ros_msg
build_v2x_if_ros_msg:
	cd "${ROOT_DIR}/v2x_if_ros_msg" && \
	make

.PHONY: build_adore_v2x_sim
build_adore_v2x_sim:
	cd "${ROOT_DIR}/adore_v2x_sim" && \
	make

.PHONY: build_coordinate_conversion
build_coordinate_conversion:
	cd "${ROOT_DIR}/coordinate_conversion" && \
	make

.PHONY: build_sumo_if_ros
build_sumo_if_ros: build_adore_if_ros_msg build_coordinate_conversion build_adore_v2x_sim
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
lint: ## Print out lint report to console
	find . -name "**lint_report.log" -exec rm -rf {} \;
	cd cpplint_docker && \
    make lint CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros) | \
	tee ${ROOT_DIR}/sumo_if_ros/sumo_if_ros_lint_report.log; exit $$PIPESTATUS

.PHONY: lintfix 
lintfix: ## Automated lint fixing of sumo_if_ros source code using clang-format
	cd cpplint_docker && \
    make lintfix CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)

.PHONY: lintfix_simulate
lintfix_simulate:
	cd cpplint_docker && \
    make lintfix_simulate CPP_PROJECT_DIRECTORY=$(realpath ${ROOT_DIR}/sumo_if_ros)


.PHONY: cppcheck 
cppcheck: ## Print out cppcheck static analysis report of the sumo_if_ros source code.
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	cd cppcheck_docker && \
    make cppcheck CPP_PROJECT_DIRECTORY=$$(realpath ${ROOT_DIR}/sumo_if_ros) | \
	tee ${ROOT_DIR}/sumo_if_ros/sumo_if_ros_cppcheck_report.log; exit $$PIPESTATUS

.PHONY: lizard 
lizard: ## Print out lizard static analysis report of the sumo_if_ros source code.
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	cd lizard_docker && \
    make lizard CPP_PROJECT_DIRECTORY=$$(realpath ${ROOT_DIR}/sumo_if_ros)
	find . -name "**lizard_report.xml" -print0 | xargs -0 -I {} mv {} sumo_if_ros/sumo_if_ros_lizard_report.xml

.PHONY: static_checks
static_checks: lizard cppcheck lint
