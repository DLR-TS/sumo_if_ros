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

RUN mkdir -p build/cmake-build && cd build/cmake-build && \
    cmake ../.. && \
    make -j$(nproc)

# copy headers to include folder
RUN mkdir -p build/cmake-build/install/include/libsumo && \
    cp src/libsumo/*.h build/cmake-build/install/include/libsumo/
#bundle o files int a static library a file
RUN cd build/cmake-build/src/libsumo/CMakeFiles/libsumostatic.dir && \
    ar rc libsumocpp.a *.o
#create lib folder and copy the a file into lib
RUN mkdir -p build/cmake-build/install/lib &&  \
    cp build/cmake-build/src/libsumo/CMakeFiles/libsumostatic.dir/*.a \
       build/cmake-build/install/lib
#copy bin files to lib folder
RUN cp bin/libsumocpp.so \ 
       build/cmake-build/install/lib

FROM alpine:3.14

ARG PROJECT
COPY --from=sumo_builder /tmp/${PROJECT}/build/cmake-build /tmp/${PROJECT}/build

