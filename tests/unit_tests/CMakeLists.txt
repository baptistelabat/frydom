

set(UNIT_TESTS
        unitTests_damping_forces
        unitTests_geographiclib_wrap
        )

# set(CMAKE_CXX_FLAGS " -lIrrlicht")

include(Add_matplotlib-cpp)


message(STATUS "Unit test programs for FRyDoM framework...")

foreach(test ${UNIT_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp)

endforeach()
