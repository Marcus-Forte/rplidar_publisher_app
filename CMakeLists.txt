cmake_minimum_required(VERSION 3.20)
project(rplidar_app)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(USE_GRPC "Publish to default gRPC port" OFF)

add_subdirectory(third_party/rplidar_sdk)
add_subdirectory(grpc)

add_executable(rplidar_publisher src/main.cc)

target_link_libraries(rplidar_publisher rplidar_sdk::rplidar_sdk grpc_publisher)
add_definitions(-DUSE_GRPC=1)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} CACHE PATH "default Install prefix" FORCE)
endif()

install(TARGETS rplidar_publisher
RUNTIME_DEPENDENCIES
POST_EXCLUDE_REGEXES ".*ld-linux.*"  # Linux dynamic linker
                    "libc.so"      # Common C libraries
                    "libgcc_"
                    "libstdc"
                    "libm.*"      # Math library
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)

set(CPACK_PACKAGE_NAME "rplidar")
set(CPACK_GENERATOR "RPM")
include(CPack)