message(STATUS "        ...Environment/field")

set(FIELD_FILES_LOCAL
        FrUniformCurrentField.cpp
        )

set(FIELD_FILES "")
foreach(file ${FIELD_FILES_LOCAL})
    list(APPEND FIELD_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FIELD_FILES ${FIELD_FILES} PARENT_SCOPE)
