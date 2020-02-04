message(STATUS "    ...tests")

message(STATUS "Test programs for FRyDoM framework...")


# Resource data loading / path definition
set(FRYDOM_DATA_FOLDER ${CMAKE_SOURCE_DIR}/data)

if (${frydom_build_dev_test})
    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/dev_tests)
    configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/dev_tests/.frydom_config)
    add_subdirectory(dev_tests)
endif ()

if (${frydom_build_unit_test})
    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/unit_tests)
    configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/unit_tests/.frydom_config)
    add_subdirectory(unit_tests)
endif ()

if (${frydom_build_demo})
    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/demo)
    configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/demo/.frydom_config)
    add_subdirectory(demos)
endif ()

