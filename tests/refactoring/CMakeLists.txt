
message(STATUS "    ...REFACTORING TESTS :")



set(REFACTORING_TESTS
        test_tug_refactoring
        )


foreach(test ${REFACTORING_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp)

endforeach()

