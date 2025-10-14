cmake_minimum_required(VERSION 3.12)

SET( MORE_FLAGS "-fPIC")
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${MORE_FLAGS}" )

#list(APPEND imbe_vocoder_sources
list(APPEND op25_repeater_sources
    aux_sub.cc
    basicop2.cc
    ch_decode.cc
    ch_encode.cc
    dc_rmv.cc
    decode.cc
    dsp_sub.cc
    encode.cc
    imbe_vocoder.cc
    math_sub.cc
    pe_lpf.cc
    pitch_est.cc
    pitch_ref.cc
    qnt_sub.cc
    rand_gen.cc
    sa_decode.cc
    sa_encode.cc
    sa_enh.cc
    tbls.cc
    uv_synt.cc
    v_synt.cc
    v_uv_det.cc
)

#add_library(imbe_vocoder SHARED ${imbe_vocoder_sources})
#set_target_properties(imbe_vocoder PROPERTIES ENABLE_EXPORTS 1)
