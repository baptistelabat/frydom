message(STATUS "        ...hydrostatics")

set(SRC_FILES
        FrHydrostaticForce.cpp
        FrLinearHydrostaticForce.cpp
        FrLinearHydrostaticStiffnessMatrix.cpp
        )

set(FRYDOM_HYDRODYNAMICS_HYDROSTATICS_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_HYDROSTATICS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_HYDRODYNAMICS_HYDROSTATICS_SOURCES ${FRYDOM_HYDRODYNAMICS_HYDROSTATICS_SOURCES} PARENT_SCOPE)
