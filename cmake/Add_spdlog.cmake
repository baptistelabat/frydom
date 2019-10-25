
include(FetchContent)

FetchContent_Declare(spdlog
        GIT_REPOSITORY ${spdlog_URL}
        GIT_TAG ${spdlog_TAG}
        )

FetchContent_GetProperties(spdlog)
if (NOT spdlog_POPULATED)
    message(STATUS "Downloading, Configuring and Generating 'spdlog' dependency")
    FetchContent_Populate(spdlog)

    # spdlog BUILD OPTIONS
    set(BUILD_APPS ON) # TODO: mettre OFF


    add_subdirectory(${spdlog_SOURCE_DIR} ${spdlog_BINARY_DIR})
else ()
    message(STATUS "spdlog already populated")
endif ()
