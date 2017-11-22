message(STATUS "    ...Cable")

set(CABLE_FILES_LOCAL
        FrCable.h
        FrCable.cpp
        FrCatenaryForce.h
        FrCatenaryForce.cpp
        FrCatenaryLine.h
        FrCatenaryLine.cpp
        FrCatenaryNode.h
        FrCatenaryNode.cpp
        FrDynamicCable.h
        FrDynamicCable.cpp
)

set(CABLE_FILES "")
foreach(file ${CABLE_FILES_LOCAL})
    list(APPEND CABLE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(CABLE_FILES ${CABLE_FILES} PARENT_SCOPE)