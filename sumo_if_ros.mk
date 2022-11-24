# This Makefile contains useful targets that can be included in downstream projects.

ifndef sumo_if_ros_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
SUMO_IF_ROS_PROJECT:=sumo_if_ros

SUMO_IF_ROS_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
MAKE_GADGETS_PATH:=${SUMO_IF_ROS_MAKEFILE_PATH}/adore_if_ros_msg/make_gadgets
REPO_DIRECTORY:=${SUMO_IF_ROS_MAKEFILE_PATH}
CPP_PROJECT_DIRECTORY:=${REPO_DIRECTORY}/${SUMO_IF_ROS_PROJECT}

SUMO_IF_ROS_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${REPO_DIRECTORY}")
SUMO_IF_ROS_IMAGE:=${SUMO_IF_ROS_PROJECT}:${SUMO_IF_ROS_TAG}

SUMO_IF_ROS_CMAKE_BUILD_PATH:="${SUMO_IF_ROS_PROJECT}/build"
SUMO_IF_ROS_CMAKE_INSTALL_PATH:="${SUMO_IF_ROS_CMAKE_BUILD_PATH}/install"

include ${SUMO_IF_ROS_MAKEFILE_PATH}/adore_v2x_sim/adore_v2x_sim.mk
include ${SUMO_IF_ROS_MAKEFILE_PATH}/coordinate_conversion/coordinate_conversion.mk
include ${SUMO_IF_ROS_MAKEFILE_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk
include ${SUMO_IF_ROS_MAKEFILE_PATH}/cpplint_docker/cpplint_docker.mk
include ${SUMO_IF_ROS_MAKEFILE_PATH}/cppcheck_docker/cppcheck_docker.mk
include ${SUMO_IF_ROS_MAKEFILE_PATH}/lizard_docker/lizard_docker.mk

.PHONY: build_sumo_if_ros 
build_sumo_if_ros: ## Build sumo_if_ros
	cd "${sumo_if_ros_MAKEFILE_PATH}" && make

.PHONY: clean_sumo_if_ros
clean_sumo_if_ros: ## Clean sumo_if_ros build artifacts
	cd "${sumo_if_ros_MAKEFILE_PATH}" && make clean

.PHONY: branch_sumo_if_ros
branch_sumo_if_ros: ## Returns the current docker safe/sanitized branch for sumo_if_ros
	@printf "%s\n" ${sumo_if_ros_tag}

.PHONY: image_sumo_if_ros
image_sumo_if_ros: ## Returns the current docker image name for sumo_if_ros
	@printf "%s\n" ${sumo_if_ros_image}

.PHONY: update_sumo_if_ros
update_sumo_if_ros:
	cd "${sumo_if_ros_MAKEFILE_PATH}" && git pull

endif
