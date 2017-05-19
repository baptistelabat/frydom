#=============================================================================
#
# CMake configuration file for the FRyDoM Project
#
#=============================================================================

#-----------------------------------------------------------------------------
# Check backward compatibility
#-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8)

#-----------------------------------------------------------------------------
# Prevent in-source builds.
#-----------------------------------------------------------------------------

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message( FATAL_ERROR  "In-source build is not possible and not reccomended. Choose an empty directory for build output.")
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
# Location of additional CMake scripts
#-----------------------------------------------------------------------------

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC VCS_REVISION)

if("${VCS_REVISION}" MATCHES ".*-NOTFOUND$")
    set(VCS_REVISION "")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)