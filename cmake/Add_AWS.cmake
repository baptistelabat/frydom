
set(langlee_URL https://frydom-ce-data.s3.amazonaws.com/demo/langlee/langlee_data.tar.gz)

set(out ${CMAKE_SOURCE_DIR}/data/Langlee/Langlee.tar.gz)
if(NOT EXISTS ${out})
    file(DOWNLOAD ${langlee_URL} ${out})
    execute_process(
            COMMAND
                ${CMAKE_COMMAND} -E tar xzf ${out}
            WORKING_DIRECTORY
                ${CMAKE_SOURCE_DIR}/data/Langlee
    )
endif()
set(langlee_resources_path ${CMAKE_SOURCE_DIR}/data/Langlee/)
