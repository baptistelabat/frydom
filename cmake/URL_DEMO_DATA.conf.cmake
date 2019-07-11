message(STATUS "      ...URL_DEMO_DATA")

set(AWS_URL https://frydom-ce-data.s3.amazonaws.com/demo)

set(DEFAULT_TAG "1.0.0")
set(Langlee_TAG "1.0.1")
set(FOSWEC_TAG "1.0.1")

foreach(test ${DEMOS})

    if (NOT ${test}_TAG)
        set(VERSION ${DEFAUL_TAG})
    else()
        set(VERSION ${${test}_TAG})
    endif()

    set(DEMO_DATA_FILE ${test}_V${VERSION}.tar.gz)

    set(OUT ${CMAKE_SOURCE_DIR}/tests/data/${test}/${DEMO_DATA_FILE})
    if (NOT EXISTS ${OUT})
        file(DOWNLOAD ${AWS_URL}/${DEMO_DATA_FILE} ${OUT} STATUS status)
        #message(STATUS "DEMO_DATA_FILE : ${AWS_URL}/${DEMO_DATA_FILE}")
        list(GET status 0 error_code)
        #message(STATUS "Downloading error status : ${error_code}")
        if( error_code )
            execute_process(
                COMMAND
                    ${CMAKE_COMMAND} -E remove_directory ${CMAKE_SOURCE_DIR}/tests/data/${test}/
                WORKING_DIRECTORY
                    ${CMAKE_SOURCE_DIR}/tests/data/
            )
        else()
            execute_process(
                COMMAND
                    ${CMAKE_COMMAND} -E tar xzf ${OUT}
                WORKING_DIRECTORY
                    ${CMAKE_SOURCE_DIR}/tests/data/${test}
            )
        endif()
    endif()
    set(${test}_resources_path ${CMAKE_SOURCE_DIR}/tests/data/${test}/)

endforeach()