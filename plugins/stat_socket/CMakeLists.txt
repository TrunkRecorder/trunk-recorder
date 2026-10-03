add_library(stat_socket 
  MODULE
    stat_socket.cc
 )

 target_link_libraries(stat_socket trunk_recorder_library ssl crypto ${Boost_LIBRARIES} ${GNURADIO_PMT_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_FILTER_LIBRARIES} ${GNURADIO_DIGITAL_LIBRARIES} ${GNURADIO_ANALOG_LIBRARIES} ${GNURADIO_AUDIO_LIBRARIES} ${TR_UHD_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES} ${GNURADIO_OSMOSDR_LIBRARIES}  ${LIBOP25_REPEATER_LIBRARIES} gnuradio-op25_repeater) # gRPC::grpc++_reflection protobuf::libprotobuf)

 if(NOT Gnuradio_VERSION VERSION_LESS "3.8")

    target_link_libraries(stat_socket
    gnuradio::gnuradio-analog
    gnuradio::gnuradio-blocks
    gnuradio::gnuradio-digital
    gnuradio::gnuradio-filter
    gnuradio::gnuradio-pmt
    ) 

endif()

install(TARGETS stat_socket LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/trunk-recorder)

