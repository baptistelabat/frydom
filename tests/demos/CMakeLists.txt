
message(STATUS "    ...DEMOS :")

# Name executable

set(DEMOS
        Ballistic
        Environment
        FreeSurface
        Body
        Rotation_and_Frame
        Hydrodynamic
        Morison
        Cable
        Hub_Installation
        Langlee
        FOSWEC
        MooringLines
        NonLinearHydrostatics
        Constraints
        Links
        HexagonalArticulatedBuoy
        )

set(BENCH
        SphereDecay
        SphereRegular
        SphereIrregular
        DTMB5512
        Cables
        NonlinearHydrostaticBox
        )

# Compile target
foreach(test ${DEMOS})
    message(STATUS "        ...add demo_${test}")
    add_executable(demo_${test} demo_${test}.cpp)
    target_link_libraries(demo_${test} FRyDoM_Engine)
endforeach()

foreach(test ${BENCH})
    message(STATUS "        ...add bench_${test}")
    add_executable(bench_${test} bench_${test}.cpp)
    target_link_libraries(bench_${test} FRyDoM_Engine)
endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()

