
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

set(FRYDOM_SRC_FILES "")
foreach(subdir ${subdirs})

    add_subdirectory(${subdir})
    string(TOUPPER FRYDOM_${subdir}_SOURCES SOURCE_FILES)
    list(APPEND FRYDOM_SRC_FILES ${${SOURCE_FILES}})

endforeach()


# FRyDoM dependencies
include(Add_chrono)
include(Add_MathUtils)
include(Add_yaml-cpp)
include(Add_HDF5)


#find_package(Irrlicht)
#list(APPEND CMAKE_CXX_FLAGS " -lIrrlicht") # TODO: voir comment faire ...
find_package(Irrlicht)



# FRyDoM main shared library
add_library(FRyDoM_Engine SHARED ${FRYDOM_SRC_FILES})
target_link_libraries(FRyDoM_Engine SHARED
        ChronoEngine
        ChronoEngine_fea
#        ChronoEngine_irrlicht
#        ChronoEngine_potprocess
        MathUtils::MathUtils
        yaml-cpp
        Irrlicht
        )


message(${CMAKE_CURRENT_SOURCE_DIR}/..)
target_include_directories(FRyDoM_Engine PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )


# Adding tests as target
#add_subdirectory(tests)
