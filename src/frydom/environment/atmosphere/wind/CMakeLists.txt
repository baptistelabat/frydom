message(STATUS "            ...wind")

set(WIND_FILES_LOCAL
        FrWind.cpp
        FrWindStandardForce.cpp
        )

set(WIND_FILES "")
foreach (file ${WIND_FILES_LOCAL})
    list(APPEND WIND_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(WIND_FILES ${WIND_FILES} PARENT_SCOPE)
