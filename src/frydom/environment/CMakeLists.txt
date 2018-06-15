message(STATUS "    ...Environment")

add_subdirectory(current)
add_subdirectory(seabed)
add_subdirectory(tidal)
add_subdirectory(waves)
add_subdirectory(wind)


set(SRC_FILES
        FrEnvironment.cpp
        FrUniformCurrentField.cpp
        FrTimeZone.cpp
        )

set(FRYDOM_ENVIRONMENT_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${CURRENT_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${SEABED_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${TIDAL_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${WAVES_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${WIND_FILES})

set(FRYDOM_ENVIRONMENT_SOURCES ${FRYDOM_ENVIRONMENT_SOURCES} PARENT_SCOPE)
