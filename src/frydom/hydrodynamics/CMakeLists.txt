message(STATUS "    ...hydrodynamics")

set(SRC_FILES
        FrFroudeKrylov.cpp
        FrBEMBody.cpp
        FrHydroDB.cpp
        FrHydrostaticForce.cpp
        FrITTC57.cpp
        FrLinearDamping.cpp
        FrLinearExcitationForce.cpp
        FrLinearHydrostaticForce.cpp
        FrLinearHydrostaticStiffnessMatrix.cpp
        FrMorrisonElement.cpp
        FrMorrisonForce.cpp
        FrRadiationForce.cpp
        FrTryalForce.cpp
        FrWaveDriftForce.cpp
        FrVelocityRecorder.cpp
        FrRadiationModel.cpp
        FrHydroMapper.cpp
        FrHydroDBLoader.cpp
)

set(FRYDOM_HYDRODYNAMICS_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_HYDRODYNAMICS_SOURCES ${FRYDOM_HYDRODYNAMICS_SOURCES} PARENT_SCOPE)
