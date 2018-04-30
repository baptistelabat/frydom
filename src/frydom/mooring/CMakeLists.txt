message(STATUS "    ...mooring")

set(SRC_FILES
#        FrMooring.h
        FrMooring.cpp
)

set(FRYDOM_MOORING_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_MOORING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_MOORING_SOURCES ${FRYDOM_MOORING_SOURCES} PARENT_SCOPE)
