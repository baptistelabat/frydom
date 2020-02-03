#-------------------------------------------------------------------------------
# Downloading test data from Amazon AWS
#-------------------------------------------------------------------------------
function(DownloadData)

    include(${CMAKE_CURRENT_SOURCE_DIR}/data/DEMO_DATA_VERSION.conf.cmake)

    message(STATUS "...Downloading data for demos and tests")

    set(AWS_URL https://frydom-ce-data.s3.amazonaws.com/demo)

    set(DATA_FILE_CE data_v${TEST_DATA_VERSION}.tar.gz)

    set(OUT ${CMAKE_SOURCE_DIR}/data/ce/${DATA_FILE_CE})

    if (NOT EXISTS ${OUT})
        file(DOWNLOAD ${AWS_URL}/${DATA_FILE_CE} ${OUT} STATUS status)
        list(GET status 0 error_code)
        if( error_code )
            execute_process(
                    COMMAND
                    ${CMAKE_COMMAND} -E remove ${OUT}
                    WORKING_DIRECTORY
                    ${CMAKE_SOURCE_DIR}/data/ce
            )
        else()
            execute_process(
                    COMMAND
                    ${CMAKE_COMMAND} -E tar xzf ${OUT}
                    WORKING_DIRECTORY
                    ${CMAKE_SOURCE_DIR}/data/ce
            )
        endif()
    endif()
endfunction()

