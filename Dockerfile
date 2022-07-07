ARG PROJECT="sumo_if_ros"

FROM adore_if_ros_msg:latest AS adore_if_ros_msg
FROM v2x_if_ros_msg:latest AS v2x_if_ros_msg
FROM sumo:v1_8_0 as sumo
FROM ros:noetic-ros-core-focal AS sumo_if_ros_builder

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"

WORKDIR /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    DEBIAN_FRONTEND=noninteractive xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY sumo/sumo /tmp/sumo

RUN mkdir -p /tmp/${PROJECT}
COPY ${PROJECT} /tmp/${PROJECT}

ENV adore_if_ros_msg_DIR=/tmp/adore_if_ros_msg/build/cmake
COPY --from=adore_if_ros_msg /tmp/adore_if_ros_msg /tmp/adore_if_ros_msg
WORKDIR /tmp/adore_if_ros_msg/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

ENV v2x_if_ros_msg_DIR=/tmp/v2x_if_ros_msg/build/cmake
COPY --from=v2x_if_ros_msg /tmp/v2x_if_ros_msg /tmp/v2x_if_ros_msg
WORKDIR /tmp/v2x_if_ros_msg/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install 


RUN cmake --install . --prefix /tmp/${PROJECT}/build/install
SHELL ["/bin/bash", "-c"]
RUN mkdir -p /tmp/${PROJECT}/build
WORKDIR /tmp/${PROJECT}/build
#RUN mkdir -p build && \
#    cd build && \
#    source /opt/ros/noetic/setup.bash && \
#    cmake .. && \
#    make -j$(nproc) || true

#RUN source /opt/ros/noetic/setup.bash && \
#    cmake .. && \
#    cmake --build . --config Release --target install -- -j $(nproc) && \
#    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . || true

#RUN cmake .. -DBUILD_adore_TESTING=ON -DCMAKE_PREFIX_PATH=install -DCMAKE_INSTALL_PREFIX:PATH=install && \
#    cmake --build . --config Release --target install -- -j $(nproc) && \
#    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t .

RUN cmake .. -DBUILD_adore_TESTING=ON -DCMAKE_PREFIX_PATH=install -DCMAKE_INSTALL_PREFIX:PATH=install && \
    cmake --build . --config Release --target install -- -j $(nproc)


#FROM alpine:3.14 AS sumo_if_ros_package

#ARG PROJECT

#COPY --from=sumo_if_ros_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build


