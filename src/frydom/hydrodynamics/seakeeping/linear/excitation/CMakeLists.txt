message(STATUS "    ...excitation")

set(SRC_FILES
        FrLinearExcitationForce.cpp
        FrNonLinearFroudeKrylovForce.cpp
        FrLinearHDBForce.cpp
        FrLinearDiffractionForce.cpp
        FrLinearFroudeKrylovForce.cpp
        )

set(FRYDOM_HYDRODYNAMICS_EXCITATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_EXCITATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(FRYDOM_HYDRODYNAMICS_EXCITATION_SOURCES ${FRYDOM_HYDRODYNAMICS_EXCITATION_SOURCES} PARENT_SCOPE)
