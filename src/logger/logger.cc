#include "logger/logger.h"

#include "spdlog/spdlog.h"

namespace tubular {

void FinalizeConsole() { spdlog::drop_all(); }

#ifdef __clang__
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

std::shared_ptr<spdlog::logger> Console() {
#ifdef ANDROID
  static std::shared_ptr<spdlog::logger> s_console =
      spdlog::android_logger("console", "example");
#else
  static std::shared_ptr<spdlog::logger> s_console =
      spdlog::stdout_color_mt("console");
#endif

  return s_console;
}

void SetLogLevel(const std::string &level_str) {
  if (level_str.compare("info") == 0) {
    spdlog::set_level(spdlog::level::info);
  } else if (level_str.compare("debug") == 0) {
    spdlog::set_level(spdlog::level::debug);
  } else if (level_str.compare("critical") == 0) {
    spdlog::set_level(spdlog::level::critical);
  } else if (level_str.compare("error") == 0) {
    spdlog::set_level(spdlog::level::err);
  } else if (level_str.compare("warn") == 0) {
    spdlog::set_level(spdlog::level::warn);
  } else if (level_str.compare("trace") == 0) {
    spdlog::set_level(spdlog::level::trace);
  }
}

}  // namespace tubular
