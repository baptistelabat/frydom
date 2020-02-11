message(STATUS "    ...hydrostatic")

set(SRC_FILES
        #        FrHydrostaticForce.cpp
        FrLinearHydrostaticForce.cpp
        FrLinearHydrostaticStiffnessMatrix.cpp
        FrNonlinearHydrostaticForce.cpp
        )

set(FRYDOM_HYDROSTATIC_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_HYDROSTATIC_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(FRYDOM_HYDROSTATIC_SOURCES ${FRYDOM_HYDROSTATIC_SOURCES} PARENT_SCOPE)
