
message(STATUS "===================================================================")
message(STATUS "Getting source files for FRyDoM...")
message(STATUS "===================================================================")


# Adding a new subdirectory to the FRyDoM root source directory must be done by adding its name here
set(subdirs
        core
        environment
        hydrodynamics
        cable
        mooring
        IO
        misc
        propeller
        utils
        )

set(FRYDOM_SRC_FILES "")
foreach(subdir ${subdirs})

    add_subdirectory(${subdir})

    string(TOUPPER ${subdir}_FILES SUBDIR_FILES)

    list(APPEND FRYDOM_SRC_FILES ${${SUBDIR_FILES}})

endforeach()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
message(STATUS ${FRYDOM_THIRDPARTY_INCLUDE_DIRS})
include_directories(${FRYDOM_THIRDPARTY_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED NO_MODULE)  # FIXME: It's to import Eigen3::Eigen... Should not exist here

add_library(FRyDoM_Engine SHARED ${FRYDOM_SRC_FILES})
target_link_libraries(FRyDoM_Engine ${FRYDOM_THIRDPARTY_LIBS} Eigen3::Eigen)

# Adding tests as target
add_subdirectory(tests)