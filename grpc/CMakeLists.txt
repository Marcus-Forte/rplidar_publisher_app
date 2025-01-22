# Sensor gRPC server

add_library(sensors_server
sensors_server.cc
sensors_client.cc
sensors_service.cc
conversions.cc)
target_link_libraries(sensors_server sensors_grpc sensors_proto IImu ILidar)
target_include_directories(sensors_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# OpenGL server grpc (https://github.com/Marcus-Forte/learning-opengl/blob/main/grpc/proto/gl_server.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/gl_server.proto)
add_library(gl_server_grpc_client
${CMAKE_CURRENT_BINARY_DIR}/gl_server.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/gl_server.grpc.pb.cc
)
target_include_directories(gl_server_grpc_client PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# Proxy - forward sensor data to openGL server
add_executable(gl_proxy 
  gl_proxy.cc
)
target_link_libraries(gl_proxy sensors_grpc sensors_proto gl_server_grpc_client)
target_include_directories(gl_proxy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS gl_proxy
DESTINATION bin)
