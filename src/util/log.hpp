#pragma once

#include <array>
#include <fmt/color.h>
#include <fmt/format.h>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>
#include <zephyr/kernel.h>

namespace util::log {

enum class Level : uint8_t { Trace = 0, Debug, Info, Warn, Error, Fatal, None };

// --- 基础配置 ---
namespace config {
#ifndef LOG_MIN_LEVEL
#ifdef NDEBUG
inline constexpr Level kMinLevel = Level::Info;
#else
inline constexpr Level kMinLevel = Level::Trace;
#endif
#else
inline constexpr Level kMinLevel = static_cast<Level>(LOG_MIN_LEVEL);
#endif

inline constexpr size_t kMaxLineLength = 256;
inline constexpr bool kUseColor = true;
} // namespace config

namespace detail {
// 编译期获取文件名
inline constexpr std::string_view basename(std::string_view path) {
  size_t last = path.find_last_of("/\\");
  return (last == std::string_view::npos) ? path : path.substr(last + 1);
}

struct LevelMeta {
  std::string_view label;
  fmt::text_style style;
};

inline const LevelMeta kLevelMeta[] = {
    {"TRC", fmt::fg(fmt::color::gray)},
    {"DBG", fmt::fg(fmt::color::cyan)},
    {"INF", fmt::fg(fmt::color::green)},
    {"WRN", fmt::fg(fmt::color::yellow) | fmt::emphasis::bold},
    {"ERR", fmt::fg(fmt::color::red) | fmt::emphasis::bold},
    {"FTL", fmt::fg(fmt::color::white) | fmt::bg(fmt::color::red) |
                fmt::emphasis::bold},
};

inline const auto kTagStyle = fmt::fg(fmt::color::plum);
inline const auto kDimStyle = fmt::fg(fmt::color::gray);

} // namespace detail

/**
 * @brief 日志器类
 */
class Logger {
  std::string_view tag_;

public:
  constexpr explicit Logger(std::string_view tag) : tag_(tag) {}

  template <Level L, typename... Args>

  void log(const char *file, int line, fmt::format_string<Args...> fmt_str,
           Args &&...args) const {
    // 1. 编译期与运行时的等级过滤
    if constexpr (L < config::kMinLevel)
      return;

    // 2. 准备整行输出缓冲区 (保证输出原子性，防止多任务交织)
    std::array<char, config::kMaxLineLength> buf;
    auto it = buf.begin();
    auto end = buf.end() - 3; // 预留 \r\n\0

    const auto &meta = detail::kLevelMeta[static_cast<uint8_t>(L)];

    if constexpr (config::kUseColor) {
      it = fmt::format_to(it, meta.style, "{:3s} ", meta.label);
      it = fmt::format_to(it, detail::kTagStyle, "[{:^8s}] ", tag_);
      it = fmt::format_to(it, detail::kDimStyle, "({}:{}) ",
                          detail::basename(file), line);
    } else {
      it = fmt::format_to(it, "{:3s} [{:^8s}] ({}:{}) ", meta.label, tag_,
                          detail::basename(file), line);
    }

    // 4. 格式化正文 (安全截断)
    auto res = fmt::format_to_n(it, std::distance(it, end), fmt_str,
                                std::forward<Args>(args)...);
    it = res.out;

    // 5. 封包发送
    *it++ = '\r';
    *it++ = '\n';
    *it = '\0';
    printk("%s", buf.data());

    // 6. 致命错误处理
    if constexpr (L == Level::Fatal) {
      while (true) {
        __asm volatile("bkpt #0");
      }
    }
  }

  template <typename... Args>
  void trace(fmt::format_string<Args...> f, Args &&...a) const {
    log<Level::Trace>(__builtin_FILE(), __builtin_LINE(), f,
                      std::forward<Args>(a)...);
  }
  template <typename... Args>
  void debug(fmt::format_string<Args...> f, Args &&...a) const {
    log<Level::Debug>(__builtin_FILE(), __builtin_LINE(), f,
                      std::forward<Args>(a)...);
  }
  template <typename... Args>
  void info(fmt::format_string<Args...> f, Args &&...a) const {
    log<Level::Info>(__builtin_FILE(), __builtin_LINE(), f,
                     std::forward<Args>(a)...);
  }
  template <typename... Args>
  void warn(fmt::format_string<Args...> f, Args &&...a) const {
    log<Level::Warn>(__builtin_FILE(), __builtin_LINE(), f,
                     std::forward<Args>(a)...);
  }
  template <typename... Args>
  void error(fmt::format_string<Args...> f, Args &&...a) const {
    log<Level::Error>(__builtin_FILE(), __builtin_LINE(), f,
                      std::forward<Args>(a)...);
  }
  template <typename... Args>
  void fatal(fmt::format_string<Args...> f, Args &&...a) const {
    log<Level::Fatal>(__builtin_FILE(), __builtin_LINE(), f,
                      std::forward<Args>(a)...);
  }
}; // namespace util::log

} // namespace util::log

// --- 全局便捷宏 (推荐使用，可自动捕获精确源文件行号) ---
#define LOG_TRACE(logger, fmt_str, ...)                                        \
  (logger).template log<util::log::Level::Trace>(__FILE__, __LINE__, fmt_str,  \
                                                 ##__VA_ARGS__)
#define LOG_DEBUG(logger, fmt_str, ...)                                        \
  (logger).template log<util::log::Level::Debug>(__FILE__, __LINE__, fmt_str,  \
                                                 ##__VA_ARGS__)
#define LOG_INFO(logger, fmt_str, ...)                                         \
  (logger).template log<util::log::Level::Info>(__FILE__, __LINE__, fmt_str,   \
                                                ##__VA_ARGS__)
#define LOG_WARN(logger, fmt_str, ...)                                         \
  (logger).template log<util::log::Level::Warn>(__FILE__, __LINE__, fmt_str,   \
                                                ##__VA_ARGS__)
#define LOG_ERROR(logger, fmt_str, ...)                                        \
  (logger).template log<util::log::Level::Error>(__FILE__, __LINE__, fmt_str,  \
                                                 ##__VA_ARGS__)
#define LOG_FATAL(logger, fmt_str, ...)                                        \
  (logger).template log<util::log::Level::Fatal>(__FILE__, __LINE__, fmt_str,  \
                                                 ##__VA_ARGS__)
