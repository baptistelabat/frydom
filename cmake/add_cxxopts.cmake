include(FetchContent)

FetchContent_Declare(cxxopts
        GIT_REPOSITORY ${cxxopts_URL}
        GIT_TAG ${cxxopts_TAG}
        )
FetchContent_GetProperties(cxxopts)
if(NOT cxxopts_POPULATED)
    FetchContent_Populate(cxxopts)

    set(cxxopts_enable_tests OFF CACHE BOOL "")
    set(cxxopts_build_binaries OFF CACHE BOOL "")

    add_subdirectory(${cxxopts_SOURCE_DIR} ${cxxopts_BINARY_DIR})
endif()
