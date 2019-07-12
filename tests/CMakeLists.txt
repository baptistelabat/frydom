message(STATUS "    ...tests")

message(STATUS "Test programs for FRyDoM framework...")

if (${frydom_build_dev_test})
    add_subdirectory(dev_tests)
endif()

if (${frydom_build_unit_test})
    add_subdirectory(unit_tests)
endif()

if (${frydom_build_demo})
    add_subdirectory(demos)
endif()

