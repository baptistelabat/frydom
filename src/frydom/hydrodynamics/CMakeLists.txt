message(STATUS "    ...hydrodynamics")

set(SRC_FILES
#        FrFroudeKrylov.h
        FrFroudeKrylov.cpp
#        FrBEMBody.h
        FrBEMBody.cpp
#        FrHydroDB.h
        FrHydroDB.cpp
#        FrHydrostaticForce.h
        FrHydrostaticForce.cpp
#        FrITTC57.h
        FrITTC57.cpp
#        FrLinearDamping.h
        FrLinearDamping.cpp
#        FrLinearExcitationForce.h
        FrLinearExcitationForce.cpp
#        FrLinearHydrostaticForce.h
        FrLinearHydrostaticForce.cpp
#        FrLinearHydrostaticStiffnessMatrix.h
        FrLinearHydrostaticStiffnessMatrix.cpp
#        FrMorrisonElement.h
        FrMorrisonElement.cpp
#        FrMorrisonForce.h
        FrMorrisonForce.cpp
#        FrRadiationForce.h
        FrRadiationForce.cpp
#        FrTryalForce.h
        FrTryalForce.cpp
#        FrWaveDriftForce.h
        FrWaveDriftForce.cpp
#        FrVelocityRecorder.h
        FrVelocityRecorder.cpp
#        FrRadiationModel.h
        FrRadiationModel.cpp
#        FrHydroMapper.h
        FrHydroMapper.cpp
#        FrHydroDBLoader.h
        FrHydroDBLoader.cpp
)

set(FRYDOM_HYDRODYNAMICS_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_HYDRODYNAMICS_SOURCES ${FRYDOM_HYDRODYNAMICS_SOURCES} PARENT_SCOPE)
