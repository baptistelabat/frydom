message(STATUS "    ...mesh")

set(MESH_FILES_LOCAL
        FrTriangleMeshConnected.h
        FrTriangleMeshConnected.cpp
        )

set(MESH_FILES "")
foreach(file ${MESH_FILES_LOCAL})
    list(APPEND MESH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(MESH_FILES ${MESH_FILES} PARENT_SCOPE)