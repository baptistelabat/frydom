
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

# FUNCTIONS
list(APPEND SRC_FILES
        functions/FrFunction.cpp
        )

# LINK
list(APPEND SRC_FILES

        link/FrLink.cpp

        link/actuators/FrAngularActuatorAngle.cpp
        link/actuators/FrAngularActuatorVelocity.cpp
        link/actuators/FrAngularActuatorTorque.cpp
        link/actuators/FrLinearActuatorPosition.cpp
        link/actuators/FrLinearActuatorVelocity.cpp
        link/actuators/FrLinearActuatorForce.cpp

        link/constraints/FrDistanceBetweenPointsConstraint.cpp
        link/constraints/FrParallelConstraint.cpp
        link/constraints/FrPerpendicularConstraint.cpp
        link/constraints/FrPlaneOnPlaneConstraint.cpp
        link/constraints/FrPointDistanceToAxisConstraint.cpp
        link/constraints/FrPointOnLineConstraint.cpp
        link/constraints/FrPointOnPlaneConstraint.cpp
        link/constraints/FrPointOnSplineConstraint.cpp

        link/links/FrCylindricalLink.cpp
        link/links/FrFixedLink.cpp
        link/links/FrFreeLink.cpp
        link/links/FrPrismaticLink.cpp
        link/links/FrRevoluteLink.cpp
        link/links/FrScrewLink.cpp
        link/links/FrSphericalLink.cpp

        )

# MATH
list(APPEND SRC_FILES
        math/FrMatrix.cpp
        math/FrVector.cpp
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
