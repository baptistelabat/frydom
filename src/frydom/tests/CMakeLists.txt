
set(FRYDOM_TESTS

        )

message(STATUS "Test programs for FRyDoM framework...")

foreach(test ${FRYDOM_TESTS})
    message(STATUS "...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} ${LIBS})

endforeach()