message(STATUS "    ...manoeuvring")

set(SRC_FILES
        FrManoeuvringDamping.cpp
        )

set(FRYDOM_MANOEUVRING_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_MANOEUVRING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_MANOEUVRING_SOURCES ${FRYDOM_MANOEUVRING_SOURCES} PARENT_SCOPE)