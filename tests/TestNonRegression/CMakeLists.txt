set(TESTNONREGRESSION
#        TNR_current_force
        )

message(STATUS "Test of non-regression for FRyDoM framework")

foreach(test ${TESTNONREGRESSION})
    message(STATUS "    ... add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine)
endforeach()

configure_file("../data/TNR_database.h5" . COPYONLY)