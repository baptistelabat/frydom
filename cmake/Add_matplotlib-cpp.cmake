
# TODO: trouver python dans Anaconda si present !!
# matplotlib-cpp depends on Python lib
find_package(PythonLibs 2.7)
#include_directories(${PYTHON_INCLUDE_DIRS})
#message(${PYTHON_LIBRARIES})

find_package(matplotlib-cpp QUIET)
if (NOT matplotlib-cpp_FOUND)
    include(FetchContent)

    set(matplotlib-cpp_URL https://github.com/lava/matplotlib-cpp.git)
    FetchContent_Declare(matplotlib-cpp
            GIT_REPOSITORY ${matplotlib-cpp_URL}
            GIT_TAG master
            )

    FetchContent_GetProperties(matplotlib-cpp)

#    message(STATUS ${matplotlib-cpp_POPULATED})
#    message(STATUS ${matplotlib-cpp_SOURCE_DIR})
#    message(STATUS ${matplotlib-cpp_BINARY_DIR})
    if(NOT matplotlib-cpp_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'matplotlib-cpp' dependency")
        FetchContent_Populate(matplotlib-cpp)

        # matplotlib-cpp BUILD OPTIONS
        # NONE

        add_library(matplotlib-cpp INTERFACE)
        target_include_directories(matplotlib-cpp INTERFACE
                ${PYTHON_INCLUDE_DIRS}
                ${matplotlib-cpp_SOURCE_DIR}
                ) # FIXME : voir a faire autre chose que iterface...
        target_link_libraries(matplotlib-cpp INTERFACE ${PYTHON_LIBRARIES}) # TODO: mettre en public ...


        #~ add_executable(generator generator.c)
        export(TARGETS matplotlib-cpp FILE matplotlib-cpp-exports.cmake)

        message(STATUS "DONE")

    else()
        message(STATUS "matplotlib-cpp ALREADY POPULATED")

    endif()
else()
    message(STATUS "matplotlib-cpp FOUND")
endif()



