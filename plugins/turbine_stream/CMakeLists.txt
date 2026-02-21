find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPUS REQUIRED opus)

# Generate C++ sources from proto file
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/opus_frame.proto)

add_library(turbine_stream
MODULE
  turbine_stream.cc
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_include_directories(turbine_stream PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${OPUS_INCLUDE_DIRS}
)

target_link_libraries(turbine_stream
  trunk_recorder_library
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  ${OPUS_LIBRARIES}
)

install(TARGETS turbine_stream LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/trunk-recorder)
