message(STATUS "    ...IO")

set(SRC_FILES
        FrHDF5.cpp
        FrLoader.cpp
        FrLogManager.cpp
)

set(FRYDOM_IO_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_IO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_IO_SOURCES ${FRYDOM_IO_SOURCES} PARENT_SCOPE)
