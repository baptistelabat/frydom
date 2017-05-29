# CMakeLists.txt file for the FRyDoM project

cmake_minimum_required(VERSION 2.8)

# Project's name
project(frydom)

# Setting the configuration of the build (Debug, Release...)
set(CMAKE_BUILD_TYPE Debug)  #TODO: See if it is really used...

# Project's C++ dialect
set(CMAKE_CXX_STANDARD 14)

# PATH TO THE CHRONO BUILD DIR INTO THE FRyDoM PROJECT (MAY CHANGE)
# Used by find_package command and has to point to the chrono directory that has the ChronoConfig.cmake file
set(Chrono_DIR "${CMAKE_CURRENT_SOURCE_DIR}/frydom/core/chrono/build/cmake")
#message(${Chrono_DIR})

# FIXME: the following does not make really sense wrt. the variable it creates... (taken from project_template...)
LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib64")
#message(${CMAKE_PREFIX_PATH})

# Find CHRONO package
find_package(Chrono
             COMPONENTS Irrlicht fea postprocess
             CONFIG)

#message(${CMAKE_PREFIX_PATH})

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


#--------------------------------------------------------------
# Add the executable from the project and specify all C++
# files in the targets.
#--------------------------------------------------------------
# TODO : faire en sorte de declarer les target dans un CMakeLists.txt separe par target !!
set(SOURCE_FILES

        src/environment/waves/FrFreeSurface.h
        src/environment/waves/FrFreeSurface.cpp
        src/environment/waves/FrFlatFreeSurface.h
        src/environment/waves/FrFlatFreeSurface.cpp
        src/misc/FrTriangleMeshConnected.h
        src/misc/FrTriangleMeshConnected.cpp

        src/core/FrOffshoreSystem.h
        src/core/FrOffshoreSystem.cpp

        src/tests/demo_FreeSurface.cpp)


# TODO: mettre dans un foreach !!
# To add a target
add_executable(demo_free_surface ${SOURCE_FILES})
add_executable(test_ch_system
        frydom/core/chrono/chrono/src/chrono/physics/ChSystemSMC.h
        src/tests/test_ch_system.cpp
        )



set_target_properties(test_ch_system PROPERTIES
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(test_ch_system ${CHRONO_LIBRARIES})
target_link_libraries(demo_free_surface ${CHRONO_LIBRARIES})

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#add_DLL_copy_command("${CHRONO_DLLS}")
