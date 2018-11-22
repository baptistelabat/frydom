message(STATUS "                ...tidal")

set(TIDAL_FILES_LOCAL
        FrTidalModel.cpp
)

set(TIDAL_FILES "")
foreach(file ${TIDAL_FILES_LOCAL})
    list(APPEND TIDAL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(TIDAL_FILES ${TIDAL_FILES} PARENT_SCOPE)
