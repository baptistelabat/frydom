
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
        DTMB5512_captive
        Cables
        )

# Resource data loading / path definition

include(URL_DEMO_DATA.conf)

set(SphereDecay_resources_path ${CMAKE_SOURCE_DIR}/tests/data/bench/sphere/)
set(SphereIrregular_resources_path ${CMAKE_SOURCE_DIR}/tests/data/bench/sphere/)
set(SphereRegular_resources_path ${CMAKE_SOURCE_DIR}/tests/data/bench/sphere/)

set(Hydrodynamic_resources_path ${CMAKE_SOURCE_DIR}/tests/data/platform/)
set(Morison_resources_path ${CMAKE_SOURCE_DIR}/tests/data/platform/)

set(NonLinearHydrostatics_resources_path ${CMAKE_SOURCE_DIR}/tests/data/Cylinder/)
set(Hub_Installation_resources_path ${CMAKE_SOURCE_DIR}/tests/data/barge/)

set(DTMB5512_captive_resources_path ${CMAKE_SOURCE_DIR}/tests/data/bench/DTMB5512/)

# Compile target
foreach(test ${DEMOS})
    message(STATUS "        ...add demo_${test}")
    add_executable(demo_${test} demo_${test}.cpp)
    target_link_libraries(demo_${test} FRyDoM_Engine)
    target_compile_definitions(demo_${test} PUBLIC RESOURCES_PATH="${${test}_resources_path}")
endforeach()

foreach(test ${BENCH})
    message(STATUS "        ...add bench_${test}")
    add_executable(bench_${test} bench_${test}.cpp)
    target_link_libraries(bench_${test} FRyDoM_Engine)
    target_compile_definitions(bench_${test} PUBLIC RESOURCES_PATH="${${test}_resources_path}")
endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()


configure_file(../data/ship/Ship.obj . COPYONLY)

