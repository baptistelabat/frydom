message(STATUS "    ...hydrodynamics")

set(HYDRODYNAMICS_FILES_LOCAL
        FrFroudeKrylov.h
        FrFroudeKrylov.cpp
        FrBEMBody.h
        FrBEMBody.cpp
        FrHydroDB.h
        FrHydroDB.cpp
        FrHydrostaticForce.h
        FrHydrostaticForce.cpp
        FrITTC57.h
        FrITTC57.cpp
        FrLinearDamping.h
        FrLinearDamping.cpp
        FrLinearExcitationForce.h
        FrLinearExcitationForce.cpp
        FrLinearHydrostaticForce.h
        FrLinearHydrostaticForce.cpp
        FrLinearHydrostaticStiffnessMatrix.h
        FrLinearHydrostaticStiffnessMatrix.cpp
        FrMorrisonElement.h
        FrMorrisonElement.cpp
        FrMorrisonForce.h
        FrMorrisonForce.cpp
        FrRadiationForce.h
        FrRadiationForce.cpp
        FrTryalForce.h
        FrTryalForce.cpp
        FrVelocityRecorder.h
        FrVelocityRecorder.cpp
        FrRadiationModel.h
        FrRadiationModel.cpp
        FrHydroMapper.h
        FrHydroMapper.cpp
        FrHydroDBLoader.h
        FrHydroDBLoader.cpp
)

set(HYDRODYNAMICS_FILES "")
foreach(file ${HYDRODYNAMICS_FILES_LOCAL})
    list(APPEND HYDRODYNAMICS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(HYDRODYNAMICS_FILES ${HYDRODYNAMICS_FILES} PARENT_SCOPE)
