# Cross-build for Raspberrypi 5
FROM ubuntu:noble

ARG TARGETARCH

RUN apt-get update && apt-get install -y git build-essential libi2c-dev cmake clangd clang-format g++-aarch64-linux-gnu rsync && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /deps

RUN git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc

RUN mkdir /deps/grpc/build && cd /deps/grpc/build && cmake -DgRPC_INSTALL=ON \
    -DgRPC_BUILD_TESTS=OFF \
    .. && make -j8 install

# Cross compile   
COPY cmake/aarch64.cmake /opt/cmake/aarch64.cmake
RUN if [ "${TARGETARCH}" != "arm64" ]; then \
mkdir /deps/grpc/build_aarch64 && cd /deps/grpc/build_aarch64 && cmake -DgRPC_INSTALL=ON \
 -DgRPC_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local/aarch64-linux-gnu \
 -DCMAKE_TOOLCHAIN_FILE=/opt/cmake/aarch64.cmake \
 .. && make -j8 install; \ 
 fi