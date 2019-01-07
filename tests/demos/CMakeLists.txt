
message(STATUS "    ...DEMOS :")



set(DEMOS
        demo_Ballistic
        demo_Environment
        demo_FreeSurface
        demo_Body
        demo_Rotation_and_Frame
        demo_Morison
        )


foreach(test ${DEMOS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp)

endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()


# Adding resource files
configure_file(../data/frydom_logo.png . COPYONLY)
configure_file(../../data/skybox/sky_dn.jpg . COPYONLY)
configure_file(../../data/skybox/sky_lf.jpg . COPYONLY)
configure_file(../../data/skybox/sky_up.jpg . COPYONLY)
