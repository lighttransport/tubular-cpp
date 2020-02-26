set(EXECUTABLE_TARGETS "")
add_executable(${BUILD_TARGET}-cli tubular-cli.cc)
list(APPEND EXECUTABLE_TARGETS ${BUILD_TARGET}-cli)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
  set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
        OUTPUT_NAME ${EXECUTABLE_TARGET}
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

  target_compile_features(${EXECUTABLE_TARGET} PRIVATE cxx_std_11)
  target_compile_options(${EXECUTABLE_TARGET}
          PRIVATE
          $<$<CXX_COMPILER_ID:Clang>: ${TUBULAR_CLANG_COMPILE_OPTIONS}>
          $<$<CXX_COMPILER_ID:GCC>: -Wall -Wextra>
          $<$<CXX_COMPILER_ID:MSVC>:/W4>)
  
  target_link_libraries(${EXECUTABLE_TARGET} PRIVATE 
                        ${BUILD_TARGET})
  
  if (USE_STACK_TRACE_LOGGER)
    target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_STACK_TRACE_LOGGER)
    target_link_libraries(${EXECUTABLE_TARGET} PRIVATE glog::glog)
  endif ()
endforeach()
