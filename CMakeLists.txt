#=============================================================================
#
# CMake configuration file for the FRyDoM project
#
#=============================================================================


#-----------------------------------------------------------------------------
# Check backward compatibility
#-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.11)

#-----------------------------------------------------------------------------
# Prevent in-source builds
#-----------------------------------------------------------------------------
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

#-----------------------------------------------------------------------------
# Define project name and specify version information
#-----------------------------------------------------------------------------
project(FRyDoM)

set(FRYDOM_VERSION_MAJOR 0)
set(FRYDOM_VERSION_MINOR 1)
set(FRYDOM_VERSION_PATCH 0)

set(FRYDOM_VERSION
        "${FRYDOM_VERSION_MAJOR}.${FRYDOM_VERSION_MINOR}.${FRYDOM_VERSION_PATCH}")


#-----------------------------------------------------------------------------
# Setting the configuration of the build (Debug, Release...)
#-----------------------------------------------------------------------------
#set(CMAKE_BUILD_TYPE Debug)

option(frydom_build_tests "Activate build tests" ON)
option(frydom_build_unit_test "Build unit tests" ON)
option(frydom_build_demo "Build demo tests" ON)
option(frydom_build_dev_test "Build dev test" ON)
option(frydom_download_demo_data "Downloading demo data" ON)

option(frydom_build_tools "Build the standalone tools" ON)

#-----------------------------------------------------------------------------
# C++ dialect
#-----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)


# Access to local cmake scripts
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#=============================================================================
# Retrieving the current Git revision
#=============================================================================
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
set(FRYDOM_FLAVOR "FRyDoM-CE (Community Edition)" CACHE STRING "The FRyDoM flavor (CE or EE)")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/frydom/version.cpp.in" "${CMAKE_CURRENT_BINARY_DIR}/src/frydom/version.cpp")

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/URL.conf.cmake)


if (${frydom_download_demo_data})
    include(${CMAKE_CURRENT_SOURCE_DIR}/data/DownloadData.cmake)
    DownloadData()
endif ()


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
include(Add_spdlog)

#=============================================================================
# Adding the FRyDoM project sources
#=============================================================================
add_subdirectory(src/frydom)

#=============================================================================
# Adding tests
#=============================================================================
if (${frydom_build_tests})
    enable_testing()
    include(Add_googletest)

    add_subdirectory(tests)
endif ()


#=============================================================================
# Adding tools
#=============================================================================
if (${frydom_build_tools})
    include(add_cxxopts)
    add_subdirectory(tools)
endif ()
