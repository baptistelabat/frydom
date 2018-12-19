message(STATUS "    ...damping")

set(SRC_FILES
        FrLinearDamping.cpp
        FrQuadraticDamping.cpp
        )

set(FRYDOM_DAMPING_SOURCES "")
foreach(file ${SRC_FILE})
    list(APPEND FRYDOM_DAMPING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_DAMPING_SOURCES ${FRYDOM_DAMPING_SOURCES} PARENT_SCOPE)
