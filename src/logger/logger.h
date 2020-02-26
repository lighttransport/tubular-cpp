#ifndef TUBULAR_LOGGER_H_
#define TUBULAR_LOGGER_H_

#include <memory>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#endif

#include "spdlog/spdlog.h"

namespace tubular {

///
/// Finalize console. Better to call this function when exiting the renderer.
///
void FinalizeConsole();

///
/// Get console for logging.
///
std::shared_ptr<spdlog::logger> Console();

///
/// Set log level
/// Supported level is "debug", "info", "critical", "warn", "error" and "trace"
///
void SetLogLevel(const std::string &level_str);

#define TUBULAR_STR_H(x) #x
#define TUBULAR_STR(x) TUBULAR_STR_H(x)
#define TUBULAR_PREFIX_MSG "[" __FILE__ ":" TUBULAR_STR(__LINE__) "] "

#define RTLOG_WARN(...) tubular::Console()->warn(TUBULAR_PREFIX_MSG __VA_ARGS__)
#define RTLOG_ERROR(...) \
  tubular::Console()->error(TUBULAR_PREFIX_MSG __VA_ARGS__)
#define RTLOG_CRITICAL(...) \
  tubular::Console()->critical(TUBULAR_PREFIX_MSG __VA_ARGS__)
#define RTLOG_INFO(...) tubular::Console()->info(TUBULAR_PREFIX_MSG __VA_ARGS__)
#define RTLOG_DEBUG(...) \
  tubular::Console()->debug(TUBULAR_PREFIX_MSG __VA_ARGS__)

#define TUBULAR_ASSERT(cond, ...)                                   \
  {                                                                 \
    if (!(cond)) {                                                  \
      tubular::Console()->critical(TUBULAR_PREFIX_MSG __VA_ARGS__); \
      abort();                                                      \
    }                                                               \
  }

}  // namespace tubular

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#endif  // TUBULAR_LOGGER_H_
