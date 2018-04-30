
message(STATUS "    ...Core")

set(SRC_FILES
#        FrObject.h
        FrObject.cpp
#        FrBody.h
        FrBody.cpp
#        FrBodyEasy.h
        FrBodyEasy.cpp
#        FrConstants.h
        FrConstants.cpp
#        FrCore.h
#        FrEulerAngles.h
        FrEulerAngles.cpp
#        FrForce.h
        FrForce.cpp
#        FrForceAsset.h
        FrForceAsset.cpp
#        FrHydroBody.h
        FrHydroBody.cpp
#        FrNode.h
        FrNode.cpp
#        FrOffshoreSystem.h
        FrOffshoreSystem.cpp
#        FrShip.h
        FrShip.cpp
)

set(FRYDOM_CORE_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

#message(${FRYDOM_CORE_SOURCES})

set(FRYDOM_CORE_SOURCES ${FRYDOM_CORE_SOURCES} PARENT_SCOPE)