message(STATUS "    ...logging")

set(SRC_FILES
        FrLogManager.cpp
        FrPathManager.cpp
        FrEventLogger.cpp
        FrLoggable.cpp
        FrTypeNames.cpp
        FrPathPolicies.cpp
        )

set(FRYDOM_LOGGING_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_LOGGING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(FRYDOM_LOGGING_SOURCES ${FRYDOM_LOGGING_SOURCES} PARENT_SCOPE)
