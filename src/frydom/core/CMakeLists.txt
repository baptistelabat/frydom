
message(STATUS "    ...Core")

set(CORE_FILES_LOCAL
        FrBody.h
        FrBody.cpp
        FrBodyEasy.h
        FrBodyEasy.cpp
        FrConstants.h
        FrConstants.cpp
        FrCore.h
        FrEulerAngles.h
        FrEulerAngles.cpp
        FrForce.h
        FrForce.cpp
        FrForceAsset.h
        FrForceAsset.cpp
        FrHydroBody.h
        FrHydroBody.cpp
        FrNode.h
        FrNode.cpp
        FrOffshoreSystem.h
        FrOffshoreSystem.cpp
        FrShip.h
        FrShip.cpp
)

set(CORE_FILES "")
foreach(file ${CORE_FILES_LOCAL})
    list(APPEND CORE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(CORE_FILES ${CORE_FILES} PARENT_SCOPE)