message(STATUS "        ...Environment/time")

set(TIME_FILES_LOCAL
        FrTimeZone.cpp
        )

set(TIME_FILES "")
foreach(file ${TIME_FILES_LOCAL})
    list(APPEND TIME_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(TIME_FILES ${TIME_FILES} PARENT_SCOPE)
