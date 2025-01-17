# Copyright 2011 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Project setup
########################################################################

cmake_minimum_required(VERSION 3.12)

project(Trunk-Recorder LANGUAGES CXX C VERSION "5.0.0")


configure_file(cmake.h.in "${PROJECT_BINARY_DIR}/cmake.h" @ONLY)
add_definitions(-DMANUAL_GITINFO="${MANUAL_GITINFO}")
include_directories("${PROJECT_BINARY_DIR}")

# Define the two required variables before including
# the source code for watching a git repository.
set(PRE_CONFIGURE_FILE "git.cc.in")
set(POST_CONFIGURE_FILE "${CMAKE_CURRENT_BINARY_DIR}/git.cc")
include(git_watcher.cmake)

# Create a library out of the compiled post-configure file.
add_library(git ${POST_CONFIGURE_FILE})
target_include_directories(git PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(git check_git)

# Select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

# Make sure our local CMake Modules path comes first
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake/Modules)

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
    #http://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
endif()

if(APPLE)
   set(CMAKE_MACOSX_RPATH ON)
endif()
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/trunk-recorder")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
#SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

########################################################################
# Install directories
########################################################################
include(FindPkgConfig)
find_package(Gnuradio REQUIRED)
include(GrVersion)

set(GR_REQUIRED_COMPONENTS RUNTIME ANALOG BLOCKS DIGITAL FILTER FFT PMT)
find_package(Gnuradio REQUIRED COMPONENTS runtime analog blocks digital filter fft)
if(Gnuradio_VERSION VERSION_LESS "3.8")
    find_package(Volk)
endif()


if(NOT CMAKE_MODULES_DIR)
  set(CMAKE_MODULES_DIR lib${LIB_SUFFIX}/cmake)
endif(NOT CMAKE_MODULES_DIR)


if(Gnuradio_VERSION VERSION_LESS "3.8")
    find_package(Volk)
endif()

if(NOT Gnuradio_FOUND)
    message(FATAL_ERROR "GnuRadio Runtime required to compile gqrx")
endif()


set(GR_RUNTIME_DIR      bin)
set(GR_LIBRARY_DIR      lib${LIB_SUFFIX})
set(GR_INCLUDE_DIR      include/howto)
set(GR_DATA_DIR         share)
set(GR_PKG_DATA_DIR     ${GR_DATA_DIR}/${CMAKE_PROJECT_NAME})
set(GR_DOC_DIR          ${GR_DATA_DIR}/doc)
set(GR_PKG_DOC_DIR      ${GR_DOC_DIR}/${CMAKE_PROJECT_NAME})
set(GR_CONF_DIR         etc)
set(GR_PKG_CONF_DIR     ${GR_CONF_DIR}/${CMAKE_PROJECT_NAME}/conf.d)
set(GR_LIBEXEC_DIR      libexec)
set(GR_PKG_LIBEXEC_DIR  ${GR_LIBEXEC_DIR}/${CMAKE_PROJECT_NAME})
set(GRC_BLOCKS_DIR      ${GR_PKG_DATA_DIR}/grc/blocks)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

########################################################################
# Find gnuradio build dependencies
########################################################################
#find_package(GnuradioRuntime)
#find_package(CppUnit)

# To run a more advanced search for GNU Radio and it's components and
# versions, use the following. Add any components required to the list
# of GR_REQUIRED_COMPONENTS (in all caps) and change "version" to the
# minimum API compatible version required.
#
# set(GR_REQUIRED_COMPONENTS RUNTIME BLOCKS FILTER ...)
# find_package(Gnuradio "version")
#if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
#    set(GR_REQUIRED_COMPONENTS ANALOG DIGITAL RUNTIME BLOCKS FILTER PMT)
#    find_package(Gnuradio)
#endif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")


math(EXPR GNURADIO_VERSION "(${Gnuradio_VERSION_MAJOR} / 10) << 20 | (${Gnuradio_VERSION_MAJOR} % 10) << 16 | (${Gnuradio_VERSION_MINOR} / 10) << 12 | (${Gnuradio_VERSION_MINOR} % 10) <<  8 | (${Gnuradio_VERSION_PATCH} / 10) <<  4 | (${Gnuradio_VERSION_PATCH} % 10) <<  0   ")
message(STATUS "GnuRadio Version: " ${GNURADIO_VERSION})



########################################################################
# Find build dependencies
########################################################################
find_package(GnuradioUHD)
find_package(GnuradioOsmosdr)
find_package(LibHackRF)
find_package(LibUHD)
find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)
if (STREAMER)
    find_package(Protobuf REQUIRED)
    find_package(GRPC REQUIRED)
endif()

########################################################################
# Setup boost
########################################################################
MESSAGE(STATUS "Configuring Boost C++ Libraries...")

# Although not required on my system, some users have linking issues without
SET(BOOST_REQUIRED_COMPONENTS
    thread
    system
    program_options
    filesystem
    log_setup
    log
    regex
    random
)

if(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

set(Boost_ADDITIONAL_VERSIONS
    "1.35.0" "1.35" "1.36.0" "1.36" "1.37.0" "1.37" "1.38.0" "1.38" "1.39.0" "1.39"
    "1.40.0" "1.40" "1.41.0" "1.41" "1.42.0" "1.42" "1.43.0" "1.43" "1.44.0" "1.44"
    "1.45.0" "1.45" "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
    "1.70.0" "1.70" "1.71.0" "1.71" "1.72.0" "1.72" "1.73.0" "1.73" "1.74.0" "1.74"
)

find_package(Boost COMPONENTS ${BOOST_REQUIRED_COMPONENTS} REQUIRED)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost required to build " ${CMAKE_PROJECT_NAME})
endif()

ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)
ADD_DEFINITIONS(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
if(NOT EXISTS /usr/include/spdlog/fmt/bundled)
  ADD_DEFINITIONS(-DSPDLOG_FMT_EXTERNAL)
endif()
add_definitions(-DGNURADIO_VERSION=${GNURADIO_VERSION})
message(STATUS "Gnuradio Version is: " ${Gnuradio_VERSION})

########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
    ${GNURADIO_OSMOSDR_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/lib
    ${CMAKE_BINARY_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib/op25_repeater/lib
    ${CMAKE_SOURCE_DIR}/lib/op25_repeater/include
    ${OPENSSL_INCLUDE_DIR}
    ${CURL_INCLUDE_DIR}
    "lib"
)
#    ${PROTOBUF_INCLUDE_DIR}
#    ${GRPC_INCLUDE_DIR}
#)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${GNURADIO_RUNTIME_LIBRARY_DIRS}
    ${GROSMOSDR_LIBRARIES_DIRS}
    ${GNURADIO_UHD_LIBRARIES_DIRS}
    ${UHD_LIBRARIES_DIRS}
    ${OPENSSL_ROOT_DIR}/lib
)

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wno-deprecated-declarations -Wno-error=deprecated-declarations -g3")

SET(CMAKE_CXX_STANDARD 17)

# The D_GLIBCXX_ASSERTIONS flag will cause a crash when an error is detected, making it easier to catch bugs
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wno-narrowing -fvisibility=hidden -fPIC -D_GLIBCXX_ASSERTIONS")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wno-narrowing -fvisibility=hidden -fPIC")


list(APPEND trunk_recorder_sources
  trunk-recorder/recorders/recorder.cc
  trunk-recorder/call_impl.cc
  trunk-recorder/formatter.cc
  trunk-recorder/source.cc
  trunk-recorder/call_conventional.cc
  trunk-recorder/systems/smartnet_trunking.cc
  trunk-recorder/systems/p25_trunking.cc
  trunk-recorder/systems/smartnet_parser.cc
  trunk-recorder/systems/p25_parser.cc
  trunk-recorder/systems/smartnet_decode.cc
  trunk-recorder/systems/system_impl.cc
  trunk-recorder/recorders/debug_recorder_impl.cc
  trunk-recorder/recorders/sigmf_recorder_impl.cc
  trunk-recorder/recorders/analog_recorder.cc
  trunk-recorder/recorders/dmr_recorder_impl.cc
  trunk-recorder/recorders/p25_recorder_impl.cc
  trunk-recorder/recorders/p25_recorder_fsk4_demod.cc
  trunk-recorder/recorders/p25_recorder_qpsk_demod.cc
  trunk-recorder/recorders/p25_recorder_decode.cc
  trunk-recorder/sources/iq_file_source.cc
  trunk-recorder/csv_helper.cc
  trunk-recorder/config.cc
  trunk-recorder/setup_systems.cc
  trunk-recorder/monitor_systems.cc
  trunk-recorder/talkgroup.cc
  trunk-recorder/talkgroups.cc
  trunk-recorder/unit_tag.cc
  trunk-recorder/unit_tags.cc
  trunk-recorder/plugin_manager/plugin_manager.cc
  trunk-recorder/call_concluder/call_concluder.cc

  lib/lfsr/lfsr.cxx
  #lib/gr-latency/latency_probe.cc
  #lib/gr-latency/latency_tagger.cc
  #lib/gr-latency-manager/lib/latency_manager_impl.cc
  #lib/gr-latency-manager/lib/tag_to_msg_impl.cc
  trunk-recorder/gr_blocks/freq_xlating_fft_filter.cc
  trunk-recorder/gr_blocks/transmission_sink.cc
  trunk-recorder/gr_blocks/decoders/fsync_decode.cc
  trunk-recorder/gr_blocks/decoders/mdc_decode.cc
  trunk-recorder/gr_blocks/decoders/star_decode.cc
  trunk-recorder/gr_blocks/decoders/signal_decoder_sink_impl.cc
  trunk-recorder/gr_blocks/decoders/tps_decoder_sink_impl.cc
  trunk-recorder/gr_blocks/decoder_wrapper_impl.cc
  trunk-recorder/gr_blocks/plugin_wrapper_impl.cc
  trunk-recorder/gr_blocks/selector_impl.cc
  trunk-recorder/gr_blocks/pwr_squelch_cc_impl.cc
  trunk-recorder/gr_blocks/squelch_base_cc_impl.cc
  trunk-recorder/gr_blocks/wavfile_gr3.8.cc
  trunk-recorder/gr_blocks/rms_agc.cc
  trunk-recorder/gr_blocks/channelizer.cc
    trunk-recorder/gr_blocks/xlat_channelizer.cc
    trunk-recorder/gr_blocks/signal_detector_cvf_impl.cc
  )



  list(APPEND trunk_recorder_headers
    trunk-recorder/call.h
    trunk-recorder/plugin_manager/plugin_api.h
    #lib/nlohmann/json.hpp
  )

  list(APPEND 3p_headers
    #lib/csv-parser/csv.hpp 
  )
set_source_files_properties(lib/lfsr/lfsr.cxx COMPILE_FLAGS "-w")



add_library(trunk_recorder_library
STATIC
  ${trunk_recorder_sources}
)

include(GNUInstallDirs)

add_subdirectory(lib/op25_repeater)

#target_compile_options(
#    imbe_vocoder PRIVATE
#    "-w"
#)

target_compile_options(
    gnuradio-op25_repeater PRIVATE
    "-w"
)

set_property(
    TARGET gnuradio-op25_repeater
    PROPERTY INSTALL_RPATH
    "${CMAKE_INSTALL_LIBDIR}/trunk-recorder"
)


set_target_properties(trunk_recorder_library PROPERTIES OUTPUT_NAME "trunk-recorder")
#set_target_properties(trunk_recorder_library PROPERTIES PUBLIC_HEADER "${trunk_recorder_headers}")
#set_target_properties(trunk_recorder_library PROPERTIES PRIVATE_HEADER "${3p_headers}")

# Weird - when I add LIBRARY before the DESTINATION, it installs in the /usr/local/lib dir on a mac
install(TARGETS trunk_recorder_library 
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/trunk-recorder
    #PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/trunk-recorder
)
    
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/trunk-recorder/" # source directory
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/trunk-recorder" # target directory
    FILES_MATCHING # install only matched files
    PATTERN "*.h" # select header files
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/json.hpp" # source file
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/trunk-recorder" # target directory
)

# Add built-in plugins located in /plugins
add_subdirectory(plugins/openmhz_uploader)
add_subdirectory(plugins/broadcastify_uploader)
add_subdirectory(plugins/unit_script)
add_subdirectory(plugins/rdioscanner_uploader)
#add_subdirectory(plugins/simplestream)

# Add user plugins located in /user_plugins
# Matching: /user_plugins/${plugin_dir}/CMakeLists.txt
file(GLOB plugin_dirs "user_plugins/*")
foreach(plugin_dir ${plugin_dirs})
  if(IS_DIRECTORY ${plugin_dir})
    file(GLOB cmakelists "${plugin_dir}/CMakeLists.txt")
    if(cmakelists)
      add_subdirectory(${plugin_dir})
      get_filename_component(dir_name ${plugin_dir} NAME)
      message(STATUS "Added user plugin: ${dir_name}")
    endif()
  endif()
endforeach()

add_executable(trunk-recorder trunk-recorder/main.cc) # ${trunk_recorder_sources})

target_link_libraries(trunk-recorder git trunk_recorder_library gnuradio-op25_repeater   ${CMAKE_DL_LIBS} ssl crypto ${CURL_LIBRARIES} ${Boost_LIBRARIES} ${GNURADIO_PMT_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_FILTER_LIBRARIES} ${GNURADIO_DIGITAL_LIBRARIES} ${GNURADIO_ANALOG_LIBRARIES} ${GNURADIO_AUDIO_LIBRARIES} ${GNURADIO_UHD_LIBRARIES} ${UHD_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES} ${GNURADIO_OSMOSDR_LIBRARIES} ) # gRPC::grpc++_reflection protobuf::libprotobuf)

#target_link_libraries(trunk-recorder PRIVATE nlohmann_json::nlohmann_json )

message(STATUS "All libraries:" ${GNURADIO_ALL_LIBRARIES})
if(NOT Gnuradio_VERSION VERSION_LESS "3.8")
    target_link_libraries(trunk-recorder
    gnuradio::gnuradio-analog
    gnuradio::gnuradio-blocks
    gnuradio::gnuradio-digital
    gnuradio::gnuradio-filter
    gnuradio::gnuradio-pmt
    )  
endif()


install(TARGETS trunk-recorder RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
