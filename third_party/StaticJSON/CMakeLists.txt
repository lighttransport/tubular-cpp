cmake_minimum_required(VERSION 3.0)
project(StaticJSON)

set (POSSIBLE_RAPIDJSON_DIR "${PROJECT_SOURCE_DIR}/rapidjson-1.1.0/include")
find_path(RAPIDJSON_INCLUDE_DIR rapidjson/rapidjson.h PATHS ${POSSIBLE_RAPIDJSON_DIR})
if (NOT RAPIDJSON_INCLUDE_DIR)
    set (RAPIDJSON_FILENAME ${PROJECT_SOURCE_DIR}/rapidjson.tgz)
    file(DOWNLOAD "https://github.com/miloyip/rapidjson/archive/v1.1.0.tar.gz" ${RAPIDJSON_FILENAME}
        SHOW_PROGRESS EXPECTED_HASH SHA256=bf7ced29704a1e696fbccf2a2b4ea068e7774fa37f6d7dd4039d0787f8bed98e)
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf ${RAPIDJSON_FILENAME} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    find_path(RAPIDJSON_INCLUDE_DIR rapidjson/rapidjson.h PATHS ${POSSIBLE_RAPIDJSON_DIR})
endif()

set(SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/staticjson.cpp)
add_library(staticjson SHARED ${SOURCE_FILES})

target_include_directories(staticjson PUBLIC SYSTEM ${RAPIDJSON_INCLUDE_DIR})
target_include_directories(staticjson PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_compile_options(staticjson PRIVATE
          $<$<CXX_COMPILER_ID:Clang>:  -Wall -Wextra -pedantic -g>
          $<$<CXX_COMPILER_ID:GCC>:    -Wall -Wextra -pedantic -g>
          $<$<CXX_COMPILER_ID:MSVC>:>)
 
target_compile_features(staticjson
                        PRIVATE cxx_std_11)   


