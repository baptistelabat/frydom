
include(FetchContent)

FetchContent_Declare(platform_folders
        GIT_REPOSITORY ${platform_folders_URL}
        GIT_TAG ${platform_folders_TAG}
        PATCH_COMMAND patch < "${PROJECT_SOURCE_DIR}/cmake/patches/${platform_folders_PATCH}"
        )

FetchContent_GetProperties(platform_folders)
if (NOT platform_folders_POPULATED)
    message(STATUS "Downloading, Configuring and Generating 'platform_folders' dependency")
    FetchContent_Populate(platform_folders)



    add_subdirectory(${platform_folders_SOURCE_DIR} ${platform_folders_BINARY_DIR})
else ()
    message(STATUS "platform_folders already populated")
endif ()


if (TARGET platform_folders)
    get_target_property(INC platform_folders PUBLIC_HEADER)
    message(STATUS "Found platform_folders : ${INC}")
else()
    message(STAUS "platform_folders target not found")
endif()
