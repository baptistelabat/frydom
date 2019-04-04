
find_package(OpenMesh QUIET)

if (NOT OpenMesh_FOUND)
    include(FetchContent)

    set(OpenMesh_URL "https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git")
    FetchContent_Declare(OpenMesh
            GIT_REPOSITORY ${OpenMesh_URL}
            GIT_TAG "OpenMesh-7.0"
            )

    FetchContent_GetProperties(OpenMesh)
    #        message(STATUS OpenMesh)
    #        message(STATUS ${openmesh_POPULATED})
    #        message(STATUS ${openmesh_SOURCE_DIR})
    #        message(STATUS ${openmesh_BINARY_DIR})
    if(NOT openmesh_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'OpenMesh' dependency")
        FetchContent_Populate(OpenMesh)

        # OpenMesh BUILD OPTIONS
        set(BUILD_APPS ON) # TODO: mettre OFF


        add_subdirectory(${openmesh_SOURCE_DIR} ${openmesh_BINARY_DIR})
    else()
        message(STATUS "OpenMesh already populated")
    endif()
endif()



if (TARGET OpenMeshCore)
    get_target_property(INC OpenMeshCore INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found OpenMesh : ${OPENMESH_INCLUDE_DIR}")

    target_include_directories(OpenMeshCore INTERFACE ${OPENMESH_INCLUDE_DIR})


else()
    message(STATUS "OpenMesh target NOT FOUND")
endif()
#
##target_include_directories(ceres INTERFACE ${OpenMesh_SOURCE_DIR}/include)
##set_target_properties(ceres PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${OpenMesh_SOURCE_DIR}/include)
#
#add_library(ceres_external INTERFACE)
#target_link_libraries(ceres_external INTERFACE ceres)
##target_include_directories(ceres_external INTERFACE ${OpenMesh_SOURCE_DIR}/include)
#
#get_target_property(CERES_INCLUDE_DIRECTORIES ceres INCLUDE_DIRECTORIES)
##message(STATUS ${INC})
#foreach(inc ${CERES_INCLUDE_DIRECTORIES})
#    target_include_directories(ceres_external INTERFACE ${inc})
#endforeach()


