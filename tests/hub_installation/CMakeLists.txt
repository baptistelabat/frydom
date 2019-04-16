


set(HUB_INSTALLATION_TESTS
        test_hub_installation
        PoC_hub_installation
        Case1_A075_T6_w025RPM
        Case2_A150_T10_w025RPM
        Case3_A075_T6_w05RPM
        )

# set(CMAKE_CXX_FLAGS " -lIrrlicht")

include(Add_matplotlib-cpp)


#message(STATUS "Test programs for FRyDoM framework...")

foreach(test ${HUB_INSTALLATION_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp)

endforeach()

configure_file(data/Barge.obj . COPYONLY)
configure_file(data/BaseCrane.obj . COPYONLY)
configure_file(data/arm_crane.obj . COPYONLY)
configure_file(data/HubBox.obj . COPYONLY)
configure_file(data/Barge_frydom.hdb5 . COPYONLY)
