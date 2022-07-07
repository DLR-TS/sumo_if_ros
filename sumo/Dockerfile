ARG PROJECT="sumo"

FROM ubuntu:20.04 AS sumo_builder

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"


WORKDIR /tmp/${PROJECT}
copy files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \                                     
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/${PROJECT}
COPY ${PROJECT} /tmp/${PROJECT}

WORKDIR /tmp/${PROJECT}
RUN mkdir -p .build && \ 
    cd .build && \
    cmake .. && \
    make -j$(nproc)

FROM alpine:3.14

ARG PROJECT
COPY --from=sumo_builder /tmp/${PROJECT}/.build /tmp/${PROJECT}/.build

