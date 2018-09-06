message(STATUS "        ...Environment/waves")

set(WAVES_FILES_LOCAL
        FrFreeSurface.cpp
        FrWaveDispersionRelation.cpp
        FrWaveField.cpp
        FrWaveProbe.cpp
        FrWaveSpectrum.cpp
        FrFlowSensor.cpp
        FrKinematicStretching.cpp
        )

set(WAVES_FILES "")
foreach(file ${WAVES_FILES_LOCAL})
    list(APPEND WAVES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(WAVES_FILES ${WAVES_FILES} PARENT_SCOPE)
