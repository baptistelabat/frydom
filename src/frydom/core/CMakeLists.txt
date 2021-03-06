
message(STATUS "    ...Core")

set(SRC_FILES
        FrOffshoreSystem.cpp
        )

# BODY
list(APPEND SRC_FILES
        body/FrBody.cpp
        body/FrBodyEasy.cpp
        body/FrInertiaTensor.cpp
        body/FrAssembly.cpp
        )

# COMMON
list(APPEND SRC_FILES
        common/FrException.cpp
        common/FrFrame.cpp
        common/FrNode.cpp
        common/FrObject.cpp
        common/FrPhysicsItem.cpp
        common/FrFEAMesh.cpp
        common/FrRotation.cpp
        )

# FORCE
list(APPEND SRC_FILES
        force/FrForce.cpp
        )

# LINK // TODO : cmakiser les repertoires !!
list(APPEND SRC_FILES

        link/FrLinkBase.cpp
        link/FrLinkMaskBase.cpp

        link/links_lib/actuators/FrActuator.cpp
        link/links_lib/actuators/FrLinearActuator.cpp
        link/links_lib/actuators/FrAngularActuator.cpp

        link/constraint/FrConstraint.cpp
        link/constraint/FrCGeometrical.cpp
        link/constraint/FrConstraintTwoBodiesBase.cpp

        link/links_lib/FrLink.cpp
        link/links_lib/FrCylindricalLink.cpp
        link/links_lib/FrFixedLink.cpp
        link/links_lib/FrFreeLink.cpp
        link/links_lib/FrPrismaticLink.cpp
        link/links_lib/FrRevoluteLink.cpp
        link/links_lib/FrScrewLink.cpp
        link/links_lib/FrSphericalLink.cpp
        link/links_lib/FrDOFMaskLink.cpp
        link/links_lib/FrPrismaticRevoluteLink.cpp
        )

# MATH
list(APPEND SRC_FILES
        math/FrMatrix.cpp
        math/FrVector.cpp
        math/Torsor.cpp

        math/functions/FrFunctionBase.cpp

        math/functions/ramp/FrLinearRampFunction.cpp
        math/functions/ramp/FrTanhRampFunction.cpp
        math/functions/ramp/FrCosRampFunction.cpp

        math/functions/lib/FrPowFunction.cpp
        math/functions/lib/FrLinearFunction.cpp
        math/functions/lib/FrSaturateFunction.cpp
        math/functions/lib/FrClampFunction.cpp
        math/functions/lib/FrPolynomialFunction.cpp
        math/functions/lib/FrTrigonometricFunctions.cpp
        math/functions/lib/FrHyperbolicTrigonometricFunctions.cpp
        math/functions/lib/FrMiscFunctions.cpp

        )

# MISC
list(APPEND SRC_FILES
        misc/FrColors.cpp
        )

# STATICS
list(APPEND SRC_FILES
        statics/FrStaticAnalysis.cpp
        ../hydrodynamics/hydrostatic/FrHydroStaticEquilibrium.cpp
        )


set(FRYDOM_CORE_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()


set(FRYDOM_CORE_SOURCES ${FRYDOM_CORE_SOURCES} PARENT_SCOPE)
