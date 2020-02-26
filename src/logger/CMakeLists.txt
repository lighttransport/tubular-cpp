# compile some libraries which use for entire project
# - rt-logger (wrap of spdlog)
add_library(my-logger SHARED logger.cc)
target_include_directories(my-logger SYSTEM PUBLIC 
  ${PROJECT_SOURCE_DIR}/src/logger
  ${PROJECT_SOURCE_DIR}/src/logger/spdlog/include/)

target_link_libraries(my-logger PUBLIC
  Threads::Threads)
