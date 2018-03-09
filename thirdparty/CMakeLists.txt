# Cmake script that set the different variables to make thirarty library accessible to the FRyDoM project
# It set the following variables in the parent scope (main CMakeLists.txt file of the project):
#   * FRYDOM_THIRPARTY_LIBS
#   * FRYDOM_THIRDPARTY_INCLUDE_DIRS

cmake_minimum_required(VERSION 3.8)  # FIXME: a retirer !!

set(FRYDOM_THIRPARTY_LIBS "")
set(FRYDOM_THIRDPARTY_INCLUDE_DIRS "")

#=============================================================================
#
# MathUtils library
#
#=============================================================================
message(STATUS "===================================================================")
message(STATUS "Finding package MathUtils...")
message(STATUS "===================================================================")
# TODO: Utiliser un FindMathUtils

set(MathUtils_DIR ${CMAKE_CURRENT_SOURCE_DIR}/MathUtils/build)
find_package(MathUtils REQUIRED)

list(APPEND FRYDOM_THIRDPARTY_INCLUDE_DIRS ${MATHUTILS_INCLUDE_DIRS})
list(APPEND FRYDOM_THIRDPARTY_LIBS ${MATHUTILS_THIRDPARTY_LIBS})

message(STATUS "INCLUDE DIRS : " ${MATHUTILS_INCLUDE_DIRS})
message(STATUS "LIBRARIES : ${MATHUTILS_THIRDPARTY_LIBS}" )

#=============================================================================
#
# YAML-CPP library
#
#=============================================================================
message(STATUS "===================================================================")
message(STATUS "Finding package YAML-CPP...")
message(STATUS "===================================================================")
# Mandatory for each new library
set(YAML_CPP_LIB ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build/libyaml-cpp.so) # FIXME: non portable
set(YAML_CPP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/include)

list(APPEND FRYDOM_THIRDPARTY_LIBS ${YAML_CPP_LIB})
list(APPEND FRYDOM_THIRDPARTY_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})

message(STATUS "INCLUDE DIRS : " ${YAML_CPP_INCLUDE_DIR})
message(STATUS "LIBRARIES : " ${YAML_CPP_LIB})


#=============================================================================
#
# HDF5 library
#
#=============================================================================
message(STATUS "===================================================================")
message(STATUS "Finding package HDF5...")
message(STATUS "===================================================================")
find_package(HDF5 REQUIRED COMPONENTS CXX)

list(APPEND FRYDOM_THIRDPARTY_LIBS ${HDF5_CXX_LIBRARIES})
list(APPEND FRYDOM_THIRDPARTY_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS})

message(STATUS "INCLUDE DIRS : " ${HDF5_INCLUDE_DIRS})
message(STATUS "LIBRARIES : " ${HDF5_CXX_LIBRARIES})

#=============================================================================
#
# Boost library
#
#=============================================================================
message(STATUS "===================================================================")
message(STATUS "Finding package Boost...")
message(STATUS "===================================================================")
# To add some boost components, we just have to append their name at the end of the following find_package
find_package(Boost REQUIRED
        filesystem
        system
        )

# Mandatory for each new library
list(APPEND FRYDOM_THIRDPARTY_LIBS ${Boost_LIBRARIES})
list(APPEND FRYDOM_THIRDPARTY_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})

message(STATUS "INCLUDE DIRS : " ${Boost_INCLUDE_DIRS})
message(STATUS "LIBRARIES : " ${Boost_LIBRARIES})

#=============================================================================
#
# Adding Chrono to the project
#
#=============================================================================
message(STATUS "===================================================================")
message(STATUS "Finding package Chrono...")
message(STATUS "===================================================================")
#-----------------------------------------------------------------------------
# PATH TO THE CHRONO BUILD DIR INTO THE FRyDoM PROJECT (MAY CHANGE)
#-----------------------------------------------------------------------------

# Used by find_package command and has to point to the chrono directory that contains the ChronoConfig.cmake file
set(Chrono_DIR "${CMAKE_CURRENT_SOURCE_DIR}/chrono_build/cmake")

# Find CHRONO package
find_package(Chrono
        COMPONENTS Irrlicht fea postprocess
        CONFIG)
#find_package(Chrono
#	COMPONENTS fea fsi postprocess
#	CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
    message("Could not find Chrono or one of its required modules")
    return()
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------
include_directories(${CHRONO_INCLUDE_DIRS})


#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    set(EXTRA_COMPILE_FLAGS "/wd4275")             # disable warnings triggered by Irrlicht
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

# Mandatory for each new library
list(APPEND FRYDOM_THIRDPARTY_LIBS ${CHRONO_LIBRARIES})
list(APPEND FRYDOM_THIRDPARTY_INCLUDE_DIRS ${CHRONO_INCLUDE_DIRS})


#include_directories(${FRYDOM_THIRDPARTY_INCLUDE_DIRS})

#message(STATUS ${FRYDOM_THIRDPARTY_LIBS})

#=============================================================================
#
# Making principal variables available at parent scope (do not change !)
#
#=============================================================================

set(FRYDOM_THIRDPARTY_LIBS ${FRYDOM_THIRDPARTY_LIBS} PARENT_SCOPE)
set(FRYDOM_THIRDPARTY_INCLUDE_DIRS ${FRYDOM_THIRDPARTY_INCLUDE_DIRS} PARENT_SCOPE)
