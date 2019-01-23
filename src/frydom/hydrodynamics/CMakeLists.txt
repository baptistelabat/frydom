message(STATUS "    ...hydrodynamics")

set(SRC_FILES
        FrBEMBody.cpp
        FrHydroDB.cpp
        FrITTCResistance.cpp
        FrLinearDamping.cpp
        FrLinearExcitationForce.cpp
        FrLinearHydrostaticForce.cpp
        FrLinearHydrostaticStiffnessMatrix.cpp
        FrRadiationForce.cpp
        FrWaveDriftForce.cpp
        FrVelocityRecorder.cpp
        FrPositionRecorder.cpp
        FrRadiationModel.cpp
        FrHydroMapper.cpp
        FrHydroDBLoader.cpp
        FrMorisonModel.cpp
        FrManoeuvringDamping.cpp
        FrQuadraticDamping.cpp
        FrMorisonForce.cpp
        FrEquilibriumFrame.cpp
)

# JUNK FIXME : a retirer
list(APPEND SRC_FILES
        junk/FrFroudeKrylov.cpp
        junk/FrHydrostaticForce.cpp
        junk/FrMorrisonForce.cpp
        junk/FrMorrisonElement.cpp
        junk/FrTryalForce.cpp
        junk/FrVariablesBEMBodyMass.cpp
        )

set(FRYDOM_HYDRODYNAMICS_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_HYDRODYNAMICS_SOURCES ${FRYDOM_HYDRODYNAMICS_SOURCES} PARENT_SCOPE)
