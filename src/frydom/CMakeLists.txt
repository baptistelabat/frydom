
message(STATUS "===================================================================")
message(STATUS "Getting source files for FRyDoM...")
message(STATUS "===================================================================")


set(FRYDOM_SRC_FILES "")

# Adding a new subdirectory to the FRyDoM root source directory must be done by adding its name here
set(subdirs
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

#SET(MUMPS_ROOT "/usr/lib/" CACHE PATH "Mumps root directory" FORCE)
include(Add_chrono)
include(Add_MathUtils)
include(Add_HDF5)

include(Add_yaml-cpp)
include(Add_GeographicLib)
include(Add_fmt)
include(Add_TimeZone)
include(Add_hermes)


if (TARGET hermes)
    get_target_property(INC hermes INCLUDE_DIRECTORIES)
    message(STATUS "hermes INCLUDE_DIRS : ${INC}")
#    target_include_directories(yaml-cpp INTERFACE ${INC})
endif()

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

# FRyDoM main shared library
add_library(FRyDoM_Engine SHARED ${FRYDOM_SRC_FILES})
target_link_libraries(FRyDoM_Engine
        ChronoEngine
        ChronoEngine_fea
        ChronoEngine_irrlicht
#        ChronoEngine_mumps
#        ChronoEngine_potprocess
        MathUtils::MathUtils
        yaml-cpp
#        Irrlicht
        HDF5
        GeographicLib
        fmt
        tz
        hermes
        )
#target_link_libraries(FRyDoM_Engine yaml-cpp)


#message(${CMAKE_CURRENT_SOURCE_DIR}/..)
target_include_directories(FRyDoM_Engine PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

get_target_property(INC FRyDoM_Engine INTERFACE_LINK_LIBRARIES)
message(STATUS "FRyDoM_INCLUDE_DIRECTORIES : ${INC}")



## Adding tests as target
#add_subdirectory(tests)

# TODO: voir si on passe ce CMakeLists.txt dans le repertoire parent pour ne pas avoir a faire .. dans l'include


