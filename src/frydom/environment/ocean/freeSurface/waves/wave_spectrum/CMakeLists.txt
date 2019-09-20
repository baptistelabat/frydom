message(STATUS "                    ...wave_spectrum")

set(WAVE_SPECTRUM_FILES_LOCAL
#        FrWaveSpectrum.cpp
        FrDirectionalModel.cpp
        )

set(WAVE_SPECTRUM_FILES "")
foreach(file ${WAVE_SPECTRUM_FILES_LOCAL})
    list(APPEND WAVE_SPECTRUM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(WAVE_SPECTRUM_FILES ${WAVE_SPECTRUM_FILES} PARENT_SCOPE)
