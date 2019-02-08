
message(STATUS "    ...Core")

set(SRC_FILES
        FrOffshoreSystem.cpp
)

# BODY
list(APPEND SRC_FILES
        body/FrBody.cpp
        body/FrBodyEasy.cpp
        body/FrInertia.cpp
        )

# COMMON
list(APPEND SRC_FILES
        common/FrConvention.cpp
        common/FrException.cpp
        common/FrFrame.cpp
        common/FrNode.cpp
        common/FrObject.cpp
        common/FrPhysicsItem.cpp
        common/FrRotation.cpp
        common/FrUnits.cpp
        )

# FORCE
list(APPEND SRC_FILES
        force/FrForce.cpp
        )

## FUNCTIONS
#list(APPEND SRC_FILES
#        functions/FrFunction.cpp
#        functions/FrRamp.cpp
#        )

# LINK // TODO : cmakiser les repertoires !!
list(APPEND SRC_FILES

        link/FrLinkBase.cpp

        link/links_lib/actuators/FrActuator.cpp
        link/links_lib/actuators/FrAngularActuator.cpp
        link/links_lib/actuators/FrAngularActuatorAngle.cpp
        link/links_lib/actuators/FrAngularActuatorVelocity.cpp
        link/links_lib/actuators/FrAngularActuatorTorque.cpp
        link/links_lib/actuators/FrLinearActuator.cpp
        link/links_lib/actuators/FrLinearActuatorPosition.cpp
        link/links_lib/actuators/FrLinearActuatorVelocity.cpp
        link/links_lib/actuators/FrLinearActuatorForce.cpp

        link/constraints/FrDistanceBetweenPointsConstraint.cpp
        link/constraints/FrParallelConstraint.cpp
        link/constraints/FrPerpendicularConstraint.cpp
        link/constraints/FrPlaneOnPlaneConstraint.cpp
        link/constraints/FrPointDistanceToAxisConstraint.cpp
        link/constraints/FrPointOnLineConstraint.cpp
        link/constraints/FrPointOnPlaneConstraint.cpp
        link/constraints/FrPointOnSplineConstraint.cpp

        link/links_lib/FrLink.cpp
        link/links_lib/FrCylindricalLink.cpp
        link/links_lib/FrFixedLink.cpp
        link/links_lib/FrFreeLink.cpp
        link/links_lib/FrPrismaticLink.cpp
        link/links_lib/FrRevoluteLink.cpp
        link/links_lib/FrScrewLink.cpp
        link/links_lib/FrSphericalLink.cpp
        )

# MATH
list(APPEND SRC_FILES
        math/FrMatrix.cpp
        math/FrVector.cpp

        math/functions/FrFunction.cpp
        math/functions/FrExpFunction.cpp
        math/functions/FrLinearRampFunction.cpp
        math/functions/FrLogFunction.cpp
        math/functions/FrSinFunction.cpp
        math/functions/FrTanFunction.cpp
        math/functions/FrTanhRampFunction.cpp
        )

# MISC
list(APPEND SRC_FILES
        misc/FrColors.cpp
        )

# JUNK FIXME : a supprimer
list(APPEND SRC_FILES
        junk/FrHydroBody.cpp
        junk/FrNodeDynamic.cpp
        junk/FrShip.cpp
        junk/FrSpringDampingForce.cpp
        junk/FrUpdatable.cpp
        )



set(FRYDOM_CORE_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

#message(${FRYDOM_CORE_SOURCES})

set(FRYDOM_CORE_SOURCES ${FRYDOM_CORE_SOURCES} PARENT_SCOPE)
