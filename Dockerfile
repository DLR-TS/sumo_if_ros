ARG PROJECT="sumo_if_ros"

FROM adore_if_ros_msg:latest AS adore_if_ros_msg
FROM adore_v2x_sim:latest AS adore_v2x_sim
FROM sumo:v1_13_0 as sumo
FROM ros:noetic-ros-core-focal AS sumo_if_ros_builder

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"

WORKDIR /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    DEBIAN_FRONTEND=noninteractive xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/${PROJECT}
COPY ${PROJECT} /tmp/${PROJECT}

COPY --from=adore_if_ros_msg /tmp/adore_if_ros_msg /tmp/adore_if_ros_msg
WORKDIR /tmp/adore_if_ros_msg/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=v2x_if_ros_msg /tmp/v2x_if_ros_msg /tmp/v2x_if_ros_msg
WORKDIR /tmp/v2x_if_ros_msg/v2x_if_ros_msg/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=adore_v2x_sim /tmp/adore_v2x_sim /tmp/adore_v2x_sim
WORKDIR /tmp/adore_v2x_sim/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=sumo /tmp/sumo /tmp/sumo
# WORKDIR /tmp/sumo/build
# RUN cmake --install . --prefix /tmp/${PROJECT}/build/install


COPY --from=coordinate_conversion /tmp/coordinate_conversion /tmp/coordinate_conversion
WORKDIR /tmp/coordinate_conversion/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install


SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/build

RUN source /opt/ros/noetic/setup.bash && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="install" && \
    cmake --build . --config Release --target install -- -j $(nproc) && \
    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
    cp -r /tmp/${PROJECT}/build/devel/lib/${PROJECT} /tmp/${PROJECT}/build/install/lib/${PROJECT}

FROM alpine:3.14 AS sumo_if_ros_package

ARG PROJECT

COPY --from=sumo_if_ros_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build


