#include <stdlib.h>

#include <iostream>

#include "io/config-loader.h"
#include "tubular.h"

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

int main(int argc, char** argv) {
  (void)argc, (void)argv;
#ifdef USE_STACK_TRACE_LOGGER
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
#endif
  const std::string config_path =
      (argc > 1) ? std::string(argv[1]) : "./config.json";
  tubular::TubularConfig tubular_config;
  {
    const bool ret = tubular::LoadConfigFromJson(config_path, &tubular_config);
    if (!ret) {
      std::cerr << "failed to load config json [" << config_path << "]"
                << std::endl;
      return EXIT_FAILURE;
    }
  }
  std::cerr << "Load config json [" << config_path << "]" << std::endl;

  tubular::XpdToObj(tubular_config);
  return EXIT_SUCCESS;
}
