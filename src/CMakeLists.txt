set(TUBULAR_SOURCES
  tubular.cc
  tubular-config.cc
  trackball.cc
  curve/curve.cc
  curve/catmul-rom-curve.cc
  curve/frenet-frame.cc
  io/load_xpd.cc
  io/tiny_obj_loader.cc
  io/tiny_xpd.cc
  io/obj_writer.cc
  io/config-loader.cc
  logger/logger.cc
  mesh/triangle-mesh.cc
)

add_library(${BUILD_TARGET}
            ${TUBULAR_SOURCES})

target_compile_features(${BUILD_TARGET}
                        PRIVATE cxx_std_11)   

target_compile_options(${BUILD_TARGET} PRIVATE
          $<$<CXX_COMPILER_ID:Clang>: ${TUBULAR_CLANG_COMPILE_OPTIONS}>
          $<$<CXX_COMPILER_ID:GCC>: -Wall -Wextra>
          $<$<CXX_COMPILER_ID:MSVC>:/W4>)
 
target_include_directories(${BUILD_TARGET} PUBLIC
                           ${PROJECT_SOURCE_DIR}/src/)

target_include_directories(${BUILD_TARGET} PRIVATE
  ${PROJECT_SOURCE_DIR}/src/logger/spdlog/include)

target_link_libraries(${BUILD_TARGET} PUBLIC
                      Threads::Threads
                      staticjson)
