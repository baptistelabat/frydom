message(STATUS "    ...second_order_drift")

set(SRC_FILES
        FrWaveDriftForce.cpp
        )

set(FRYDOM_HYDRODYNAMICS_SECONDORDERDRIFT_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_SECONDORDERDRIFT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(FRYDOM_HYDRODYNAMICS_SECONDORDERDRIFT_SOURCES ${FRYDOM_HYDRODYNAMICS_SECONDORDERDRIFT_SOURCES} PARENT_SCOPE)