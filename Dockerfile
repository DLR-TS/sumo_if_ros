ARG PROJECT="sumo_if_ros"

ARG ADORE_IF_ROS_MSG_TAG="latest"
ARG V2X_IF_ROS_MSG_TAG="latest"
ARG ADORE_V2X_SIM_TAG="latest"
ARG COORDINATE_CONVERSION_TAG="latest"
ARG SUMO_TAG="latest"

FROM adore_if_ros_msg:${ADORE_IF_ROS_MSG_TAG} AS adore_if_ros_msg
FROM v2x_if_ros_msg:${V2X_IF_ROS_MSG_TAG} AS v2x_if_ros_msg
FROM adore_v2x_sim:${ADORE_V2X_SIM_TAG} AS adore_v2x_sim
FROM coordinate_conversion:${COORDINATE_CONVERSION_TAG} AS coordinate_conversion
FROM sumo:${SUMO_TAG} as sumo

FROM ros:noetic-ros-core-focal AS sumo_if_ros_requirements_base

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"

RUN mkdir -p /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    DEBIAN_FRONTEND=noninteractive xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}

ARG INSTALL_PREFIX=/tmp/${PROJECT}/${PROJECT}/build/install
RUN mkdir -p "${INSTALL_PREFIX}"

COPY --from=adore_if_ros_msg /tmp/adore_if_ros_msg /tmp/adore_if_ros_msg
WORKDIR /tmp/adore_if_ros_msg/adore_if_ros_msg/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY --from=v2x_if_ros_msg /tmp/v2x_if_ros_msg /tmp/v2x_if_ros_msg
WORKDIR /tmp/v2x_if_ros_msg/v2x_if_ros_msg/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY --from=adore_v2x_sim /tmp/adore_v2x_sim /tmp/adore_v2x_sim
WORKDIR /tmp/adore_v2x_sim/adore_v2x_sim/build
RUN cmake --install . --prefix ${INSTALL_PREFIX}


COPY --from=sumo /tmp/sumo/build /tmp/sumo/build
#WORKDIR /tmp/sumo/build
#RUN cmake .. && \
 #   cmake --install . --prefix ${INSTALL_PREFIX}

COPY --from=coordinate_conversion /tmp/coordinate_conversion /tmp/coordinate_conversion
WORKDIR /tmp/coordinate_conversion/coordinate_conversion/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY ${PROJECT} /tmp/${PROJECT}

FROM sumo_if_ros_requirements_base AS sumo_if_ros_builder

ARG PROJECT
WORKDIR /tmp/${PROJECT}/${PROJECT}
RUN mkdir -p build 

SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${PROJECT}/build

RUN source /opt/ros/noetic/setup.bash && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="install" && \
    cmake --build . --config Release --target install -- -j $(nproc) && \
    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
    cp -r /tmp/${PROJECT}/${PROJECT}/build/devel/lib/${PROJECT} /tmp/${PROJECT}/${PROJECT}/build/install/lib/${PROJECT} && \
    mv CMakeCache.txt CMakeCache.txt.build || true

#FROM alpine:3.14 AS sumo_if_ros_package

#ARG PROJECT
#COPY --from=sumo_if_ros_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build
#COPY --from=sumo_if_ros_builder /tmp/${PROJECT}/${PROJECT} /tmp/${PROJECT}/${PROJECT}

