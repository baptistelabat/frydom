
message(STATUS "===================================================================")
message(STATUS "Getting source files for FRyDoM...")
message(STATUS "===================================================================")


set(FRYDOM_SRC_FILES "")

# Adding a new subdirectory to the FRyDoM root source directory must be done by adding its name here
set(subdirs
        asset
        cable
        core
        environment
        hydrodynamics
        IO
        mesh
        mooring
        utils
        )

# TODO: reposer plutot sur target_sources()
# voir crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/

set(FRYDOM_SRC_FILES "")
foreach(subdir ${subdirs})

    add_subdirectory(${subdir})
    string(TOUPPER FRYDOM_${subdir}_SOURCES SOURCE_FILES)
    list(APPEND FRYDOM_SRC_FILES ${${SOURCE_FILES}})

endforeach()

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

include(URL.conf)

include(Add_googletest)

include(Add_chrono)
include(Add_MathUtils)
include(Add_HDF5)

include(Add_GeographicLib)
include(Add_fmt)
include(Add_cppfs)
include(Add_TimeZone)
include(Add_hermes)
include(Add_OpenMesh)
include(Add_json)

#if (TARGET hermes)
#    get_target_property(INC hermes INCLUDE_DIRECTORIES)
#    message(STATUS "hermes INCLUDE_DIRS : ${INC}")
#endif()

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

# FRyDoM main shared library
add_library(FRyDoM_Engine SHARED ${FRYDOM_SRC_FILES})
target_link_libraries(FRyDoM_Engine
        ChronoEngine
        ChronoEngine_irrlicht
        MathUtils::MathUtils
        HDF5
        GeographicLib
        fmt
        cppfs
        tz
        hermes
	OpenMeshCore
	nlohmann_json
        )

target_include_directories(FRyDoM_Engine PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

get_target_property(INC FRyDoM_Engine INTERFACE_LINK_LIBRARIES)
message(STATUS "FRyDoM_INCLUDE_DIRECTORIES : ${INC}")


# TODO: voir si on passe ce CMakeLists.txt dans le repertoire parent pour ne pas avoir a faire .. dans l'include
