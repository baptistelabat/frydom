message(STATUS "    ...mooring")

set(MOORING_FILES_LOCAL
        FrMooring.h
        FrMooring.cpp
)

set(MOORING_FILES "")
foreach(file ${MOORING_FILES_LOCAL})
    list(APPEND MOORING_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(MOORING_FILES ${MOORING_FILES} PARENT_SCOPE)
