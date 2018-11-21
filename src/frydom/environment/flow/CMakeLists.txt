message(STATUS "        ...Environment/flow")

set(FLOW_FILES_LOCAL
        FrFlowBase.cpp
        FrFlowForce.cpp
        )

set(FLOW_FILES "")
foreach(file ${FLOW_FILES_LOCAL})
    list(APPEND FLOW_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FLOW_FILES ${FLOW_FILES} PARENT_SCOPE)
