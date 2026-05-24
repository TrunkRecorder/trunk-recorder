add_library(bench_logger
MODULE
  bench_logger.cc
)

target_link_libraries(bench_logger trunk_recorder_library ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES})

if(NOT Gnuradio_VERSION VERSION_LESS "3.8")
    target_link_libraries(bench_logger
        gnuradio::gnuradio-runtime
    )
endif()

install(TARGETS bench_logger LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/trunk-recorder)
