message(STATUS ================================================)
find_package(TimeZone QUIET)
if (NOT TimeZone_FOUND)
    include(FetchContent)

    set(TimeZone_URL https://github.com/HowardHinnant/date.git)
    FetchContent_Declare(TimeZone
            GIT_REPOSITORY ${TimeZone_URL}
            GIT_TAG v2.4.1
            )

    FetchContent_GetProperties(TimeZone)
    message(STATUS TimeZone)
    message(STATUS ${timezone_POPULATED})
    message(STATUS ${timezone_SOURCE_DIR})
    message(STATUS ${timezone_BINARY_DIR})
    if(NOT timezone_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'TimeZone' dependency")
        FetchContent_Populate(TimeZone)

        # TimeZone BUILD OPTIONS
        set(USE_SYSTEM_TZ_DB OFF CACHE BOOL "Use the operating system's timezone database" FORCE)
        set(USE_TZ_DB_IN_DOT OFF CACHE BOOL "Save the timezone database in the current folder" FORCE)
        set(BUILD_SHARED_LIBS ON CACHE BOOL "Build a shared version of library" FORCE)
        set(ENABLE_DATE_TESTING OFF CACHE BOOL "Enable unit tests" FORCE)

        message(STATUS ${timezone_SOURCE_DIR})
        message(STATUS ${timezone_BINARY_DIR})

        add_subdirectory(${timezone_SOURCE_DIR} ${timezone_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()

if (TARGET tz)

    get_target_property(DIR tz INCLUDE_DIRECTORIES)
    message("Found TimeZone : ${DIR}")

endif()
message(STATUS ================================================)