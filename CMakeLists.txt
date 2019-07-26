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
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message( FATAL_ERROR  "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

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

#-----------------------------------------------------------------------------
# C++ dialect
#-----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)


# Access to local cmake scripts
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



#=============================================================================
# Adding Thirparty libraries to the project
#=============================================================================
# It set FRYDOM_THIRDPARTY_LIBS and FRYDOM_THIRDPARTY_INCLUDE_DIRS
#add_subdirectory(thirdparty)


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
endif()
