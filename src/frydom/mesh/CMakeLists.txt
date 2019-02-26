message(STATUS "    ...mesh")

set(SRC_FILES
        FrTriangleMeshConnected.cpp
	FrCache.cpp
	FrMesh.cpp
	FrTriangleIntegrations.cpp
	FrHydrostaticsProperties.cpp
	FrMeshClipper.cpp
        )

set(FRYDOM_MESH_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_MESH_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_MESH_SOURCES ${FRYDOM_MESH_SOURCES} PARENT_SCOPE)