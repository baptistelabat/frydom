message(STATUS "      ...URL_DEMO_DATA")

set(AWS_URL https://frydom-ce-data.s3.amazonaws.com/demo)
set(VERSION "1.0.1")
set(DATA_FILE data_v${VERSION}.tar.gz)

set(OUT ${CMAKE_SOURCE_DIR}/tests/data/data_v${VERSION}.tar.gz)

if (NOT EXISTS ${OUT})
    file(DOWNLOAD ${AWS_URL}/${DATA_FILE} ${OUT} STATUS status)
    list(GET status 0 error_code)
    if( error_code )
        execute_process(
            COMMAND
                ${CMAKE_COMMAND} -E remove ${OUT}
            WORKING_DIRECTORY
                ${CMAKE_SOURCE_DIR}/tests/data/
        )
    else()
        execute_process(
            COMMAND
                ${CMAKE_COMMAND} -E tar xzf ${OUT}
            WORKING_DIRECTORY
                ${CMAKE_SOURCE_DIR}/tests/data
        )
    endif()
endif()
