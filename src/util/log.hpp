#pragma once

#include <array>
#include <cstring>
#include <string_view>
#include <type_traits>
#include <utility>

#include <SEGGER_RTT.h>
#include <fmt/format.h>
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
    inline constexpr bool kUseColor        = true;
} // namespace config

namespace detail {
    inline constexpr std::string_view basename(std::string_view path) {
        size_t last = path.find_last_of("/\\");
        return (last == std::string_view::npos) ? path : path.substr(last + 1);
    }

    struct LevelMeta {
        std::string_view label;
        const char* color;
    };

    inline constexpr LevelMeta kLevelMeta[] = {
        { "TRC", RTT_CTRL_TEXT_BRIGHT_BLACK },
        { "DBG", RTT_CTRL_TEXT_BRIGHT_CYAN },
        { "INF", RTT_CTRL_TEXT_BRIGHT_GREEN },
        { "WRN", RTT_CTRL_TEXT_BRIGHT_YELLOW },
        { "ERR", RTT_CTRL_TEXT_BRIGHT_RED },
        { "FTL", RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE },
    };

    inline constexpr const char* kTagColor = RTT_CTRL_TEXT_BRIGHT_MAGENTA;
    inline constexpr const char* kDimColor = RTT_CTRL_TEXT_BRIGHT_BLACK;
    inline constexpr const char* kReset    = RTT_CTRL_RESET;

} // namespace detail

class Logger {
    std::string_view tag_;

public:
    constexpr explicit Logger(std::string_view tag)
        : tag_(tag) { }

    template <Level L, typename... Args>
    void log(
        const char* file, int line, fmt::format_string<Args...> fmt_str, Args&&... args) const {
        if constexpr (L < config::kMinLevel) return;

        std::array<char, config::kMaxLineLength> buf { };
        auto it = buf.begin();

        const auto& meta = detail::kLevelMeta[static_cast<uint8_t>(L)];

        if constexpr (config::kUseColor) {
            it = fmt::format_to(it, "{}{:3s} {} ", meta.color, meta.label, detail::kTagColor);
            it = fmt::format_to(it, "[{:^8s}] ", tag_);
            it = fmt::format_to(it, "{}({}:{}) ", detail::kDimColor, detail::basename(file), line);
        } else {
            it = fmt::format_to(
                it, "{:3s} [{:^8s}] ({}:{}) ", meta.label, tag_, detail::basename(file), line);
        }

        it = fmt::format_to(it, fmt_str, std::forward<Args>(args)...);
        it = fmt::format_to(it, "{}\r\n", detail::kReset);

        // 确保 null 终止
        size_t n = std::min(static_cast<size_t>(it - buf.begin()), buf.size() - 1);
        buf[n]   = '\0';
        printk("%s", buf.data());

        if constexpr (L == Level::Fatal) {
            while (true) {
                __asm volatile("bkpt #0");
            }
        }
    }

    template <typename... Args>
    void trace(fmt::format_string<Args...> f, Args&&... a) const {
        log<Level::Trace>(__builtin_FILE(), __builtin_LINE(), f, std::forward<Args>(a)...);
    }
    template <typename... Args>
    void debug(fmt::format_string<Args...> f, Args&&... a) const {
        log<Level::Debug>(__builtin_FILE(), __builtin_LINE(), f, std::forward<Args>(a)...);
    }
    template <typename... Args>
    void info(fmt::format_string<Args...> f, Args&&... a) const {
        log<Level::Info>(__builtin_FILE(), __builtin_LINE(), f, std::forward<Args>(a)...);
    }
    template <typename... Args>
    void warn(fmt::format_string<Args...> f, Args&&... a) const {
        log<Level::Warn>(__builtin_FILE(), __builtin_LINE(), f, std::forward<Args>(a)...);
    }
    template <typename... Args>
    void error(fmt::format_string<Args...> f, Args&&... a) const {
        log<Level::Error>(__builtin_FILE(), __builtin_LINE(), f, std::forward<Args>(a)...);
    }
    template <typename... Args>
    void fatal(fmt::format_string<Args...> f, Args&&... a) const {
        log<Level::Fatal>(__builtin_FILE(), __builtin_LINE(), f, std::forward<Args>(a)...);
    }
};

} // namespace util::log

#define LOG_TRACE(logger, fmt_str, ...)                                                            \
    (logger).template log<util::log::Level::Trace>(__FILE__, __LINE__, fmt_str, ##__VA_ARGS__)
#define LOG_DEBUG(logger, fmt_str, ...)                                                            \
    (logger).template log<util::log::Level::Debug>(__FILE__, __LINE__, fmt_str, ##__VA_ARGS__)
#define LOG_INFO(logger, fmt_str, ...)                                                             \
    (logger).template log<util::log::Level::Info>(__FILE__, __LINE__, fmt_str, ##__VA_ARGS__)
#define LOG_WARN(logger, fmt_str, ...)                                                             \
    (logger).template log<util::log::Level::Warn>(__FILE__, __LINE__, fmt_str, ##__VA_ARGS__)
#define LOG_ERROR(logger, fmt_str, ...)                                                            \
    (logger).template log<util::log::Level::Error>(__FILE__, __LINE__, fmt_str, ##__VA_ARGS__)
#define LOG_FATAL(logger, fmt_str, ...)                                                            \
    (logger).template log<util::log::Level::Fatal>(__FILE__, __LINE__, fmt_str, ##__VA_ARGS__)
