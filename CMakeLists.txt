#=============================================================================
#
# CMake configuration file for the FRyDoM project
#
#=============================================================================


#-----------------------------------------------------------------------------
# Check backward compatibility
#-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8)

#-----------------------------------------------------------------------------
# Prevent in-source builds
#-----------------------------------------------------------------------------
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message( FATAL_ERROR  "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

#-----------------------------------------------------------------------------
# Define project name and specify version information
#-----------------------------------------------------------------------------
project(frydom)

set(FRYDOM_VERSION_MAJOR 0)
set(FRYDOM_VERSION_MINOR 0)
set(FRYDOM_VERSION_PATCH 0)

set(FRYDOM_VERSION
        "${FRYDOM_VERSION_MAJOR}.${FRYDOM_VERSION_MINOR}.${FRYDOM_VERSION_PATCH}")


#-----------------------------------------------------------------------------
# Setting the configuration of the build (Debug, Release...)
#-----------------------------------------------------------------------------
set(CMAKE_BUILD_TYPE Debug)

#-----------------------------------------------------------------------------
# C++ dialect
#-----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)

#-----------------------------------------------------------------------------
# Ensuring third party software is build
#-----------------------------------------------------------------------------
#message(STATUS "Ensuring that the thirdparty software are up-to-date and properly built")
#execute_process(COMMAND python
#        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build_thirdparty.py ${CMAKE_BUILD_TYPE}
#        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
#        )

#=============================================================================
#
# Adding Thirparty libraries to the project
#
#=============================================================================
# It set FRYDOM_THIRDPARTY_LIBS and FRYDOM_THIRDPARTY_INCLUDE_DIRS
add_subdirectory(thirdparty)

#=============================================================================
#
# Adding the FRyDoM project sources
#
#=============================================================================
add_subdirectory(src/frydom)

