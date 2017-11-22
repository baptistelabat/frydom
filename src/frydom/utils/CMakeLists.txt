message(STATUS "    ...utils")

set(UTILS_FILES_LOCAL
        FrIrrApp.h
        FrIrrApp.cpp
)

set(UTILS_FILES "")
foreach(file ${UTILS_FILES_LOCAL})
    list(APPEND UTILS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(UTILS_FILES ${UTILS_FILES} PARENT_SCOPE)
