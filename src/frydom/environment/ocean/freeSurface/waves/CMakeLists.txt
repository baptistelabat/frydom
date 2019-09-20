message(STATUS "                ...waves")

add_subdirectory(airy)
add_subdirectory(wave_spectrum)

set(WAVES_FILES_LOCAL
        FrWaveDispersionRelation.cpp
        FrWaveField.cpp
        )

set(WAVES_FILES "")
foreach(file ${WAVES_FILES_LOCAL})
    list(APPEND WAVES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND WAVES_FILES ${AIRY_FILES})
list(APPEND WAVES_FILES ${WAVE_SPECTRUM_FILES})

set(WAVES_FILES ${WAVES_FILES} PARENT_SCOPE)
