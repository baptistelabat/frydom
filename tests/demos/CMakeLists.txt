
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
        NonlinearHydrostaticBox
        )

# Resource data loading / path definition

if(${frydom_download_demo_data})
    include(URL_DEMO_DATA.conf)
endif()

set(SphereDecay_resources_path ${sphere_resources_path})
set(SphereIrregular_resources_path ${sphere_resources_path})
set(SphereRegular_resources_path ${sphere_resources_path})
set(Hydrodynamic_resources_path ${platform_resources_path})
set(Morison_resources_path ${platform_resources_path})
set(NonLinearHydrostatics_resources_path ${cylinder_resources_path})
set(Hub_Installation_resources_path ${barge_resources_path})
set(DTMB5512_captive_resources_path ${DTMB5512_resources_path})

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

