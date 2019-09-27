message(STATUS "    ...logging")

set(SRC_FILES
        FrLoggable.cpp
        FrLogManager.cpp
        FrPathManager.cpp
        )

set(FRYDOM_LOGGING_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_LOGGING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_LOGGING_SOURCES ${FRYDOM_LOGGING_SOURCES} PARENT_SCOPE)
