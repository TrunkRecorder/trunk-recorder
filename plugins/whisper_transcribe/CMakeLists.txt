add_library(whisper_transcribe
        MODULE
        whisper_transcribe.cc
)

target_link_libraries(whisper_transcribe
        trunk_recorder_library
        ssl
        crypto
        ${CURL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${GNURADIO_PMT_LIBRARIES}
        ${GNURADIO_RUNTIME_LIBRARIES}
        ${GNURADIO_FILTER_LIBRARIES}
        ${GNURADIO_DIGITAL_LIBRARIES}
        ${GNURADIO_ANALOG_LIBRARIES}
        ${GNURADIO_AUDIO_LIBRARIES}
        ${GNURADIO_UHD_LIBRARIES}
        ${UHD_LIBRARIES}
        ${GNURADIO_BLOCKS_LIBRARIES}
        ${GNURADIO_OSMOSDR_LIBRARIES}
        ${LIBOP25_REPEATER_LIBRARIES}
        gnuradio-op25_repeater
)

if(NOT Gnuradio_VERSION VERSION_LESS "3.8")
    target_link_libraries(whisper_transcribe
            gnuradio::gnuradio-analog
            gnuradio::gnuradio-blocks
            gnuradio::gnuradio-digital
            gnuradio::gnuradio-filter
            gnuradio::gnuradio-pmt
    )
endif()

install(TARGETS whisper_transcribe
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/trunk-recorder
)