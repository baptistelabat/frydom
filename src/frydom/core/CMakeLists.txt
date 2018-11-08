
message(STATUS "    ...Core")

set(SRC_FILES
        FrObject.cpp
        FrUnits.cpp
        FrVector.cpp
        FrMatrix.cpp
        FrRotation.cpp
        FrFrame.cpp
        FrInertia.cpp
        FrBody.cpp
        FrLink.cpp
        FrBodyEasy.cpp
        FrColors.cpp
        FrGeographic.cpp
        FrForce.cpp
        FrForceAsset.cpp
        FrHydroBody.cpp
        FrNode.cpp
        FrOffshoreSystem.cpp
        FrShip.cpp
        FrSpringDampingForce.cpp
        FrNodeDynamic.cpp
        FrFunction.cpp
)

set(FRYDOM_CORE_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

#message(${FRYDOM_CORE_SOURCES})

set(FRYDOM_CORE_SOURCES ${FRYDOM_CORE_SOURCES} PARENT_SCOPE)