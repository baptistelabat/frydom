message(STATUS "    ...tests")

message(STATUS "Test programs for FRyDoM framework...")

include(Add_AWS)

add_subdirectory(dev_tests)
add_subdirectory(unit_tests)
add_subdirectory(demos)


