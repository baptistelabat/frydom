message(STATUS "                ...waves")

add_subdirectory(airy)

set(WAVES_FILES_LOCAL
        FrWaveDispersionRelation.cpp
        FrWaveField.cpp
        FrWaveSpectrum.cpp
        FrKinematicStretching.cpp
        )

set(WAVES_FILES "")
foreach (file ${WAVES_FILES_LOCAL})
    list(APPEND WAVES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

list(APPEND WAVES_FILES ${AIRY_FILES})

set(WAVES_FILES ${WAVES_FILES} PARENT_SCOPE)
