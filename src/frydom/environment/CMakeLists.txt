message(STATUS "    ...Environment")

add_subdirectory(current)
add_subdirectory(seabed)
add_subdirectory(tidal)
add_subdirectory(waves)
add_subdirectory(wind)


set(ENVIRONMENT_FILES_LOCAL
        FrConventions.h
        FrEnvironment.h
        FrEnvironment.cpp
        )

set(ENVIRONMENT_FILES "")
foreach(file ${ENVIRONMENT_FILES_LOCAL})
    list(APPEND ENVIRONMENT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND ENVIRONMENT_FILES ${CURRENT_FILES})
list(APPEND ENVIRONMENT_FILES ${SEABED_FILES})
list(APPEND ENVIRONMENT_FILES ${TIDAL_FILES})
list(APPEND ENVIRONMENT_FILES ${WAVES_FILES})
list(APPEND ENVIRONMENT_FILES ${WIND_FILES})

set(ENVIRONMENT_FILES ${ENVIRONMENT_FILES} PARENT_SCOPE)
