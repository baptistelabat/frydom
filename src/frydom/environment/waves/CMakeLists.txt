message(STATUS "        ...Environment/waves")

set(WAVES_FILES_LOCAL
#        FrFreeSurface.h
        FrFreeSurface.cpp
#        FrWaveDispersionRelation.h
        FrWaveDispersionRelation.cpp
#        FrWaveField.h
        FrWaveField.cpp
#        FrWaveProbe.h
        FrWaveProbe.cpp
#        FrWaveSpectrum.h
        FrWaveSpectrum.cpp
)

set(WAVES_FILES "")
foreach(file ${WAVES_FILES_LOCAL})
    list(APPEND WAVES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(WAVES_FILES ${WAVES_FILES} PARENT_SCOPE)
