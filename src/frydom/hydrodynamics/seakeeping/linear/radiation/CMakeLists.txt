message(STATUS "    ...radiation")

set(SRC_FILES
        FrRadiationForce.cpp
        FrRadiationModel.cpp
        FrVariablesBEMBodyBase.cpp
        FrRadiationModelBase.cpp
        )

set(FRYDOM_HYDRODYNAMICS_RADIATION_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_RADIATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_HYDRODYNAMICS_RADIATION_SOURCES ${FRYDOM_HYDRODYNAMICS_RADIATION_SOURCES} PARENT_SCOPE)