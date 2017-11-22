message(STATUS "    ...IO")

set(IO_FILES_LOCAL
        FrHDF5.h
        FrHDF5.cpp
        FrLoader.h
        FrLoader.cpp
)

set(IO_FILES "")
foreach(file ${IO_FILES_LOCAL})
    list(APPEND IO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(IO_FILES ${IO_FILES} PARENT_SCOPE)
