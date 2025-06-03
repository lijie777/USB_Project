/**
 * @file MCLogger.h
 * @brief 日志接口，支持输出文件及控制台
 * @author 李杰
 * @date 2025.2.24
 */

#ifndef KALOG_H
#define KALOG_H

#include <iostream>
#include <memory>
#include <mutex>

#include "LogQstring.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#ifdef _WIN32
#ifdef KALOG_EXPORTS
#define LOGDLL_API __declspec(dllexport)
#else
#define LOGDLL_API __declspec(dllimport)
#endif
#else
#define LOGDLL_API __attribute__((visibility("default")))
#endif

#ifdef _WIN32
// strrchr:查找字符在指定字符串从右面开始的第一次出现的位置，如果成功，返回该字符以及后面的字符，如果失败，返回NULL
// strcgr:查找字符在指定字符串首次出现的位置
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1) : __FILE__)
#else
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)
#endif  //_WIN32

#ifndef SUFFIX
//在警告和错误级别的日志后面追加文件名，函数名，行号
#define SUFFIX(msg)                       \
    std::string(msg)                      \
        .append("  <")                    \
        .append(__FILENAME__)             \
        .append("> <")                    \
        .append(__FUNCTION__)             \
        .append("> <")                    \
        .append(std::to_string(__LINE__)) \
        .append(">")                      \
        .c_str()
#endif  // suffix

//文件输出
#define LOG_TRACE(...) MCLogger::getInstance().GetLogger()->trace(__VA_ARGS__)
#define LOG_DEBUG(...) MCLogger::getInstance().GetLogger()->debug(__VA_ARGS__)
#define LOG_INFO(...) MCLogger::getInstance().GetLogger()->info(__VA_ARGS__)
//在警告和错误级别的日志后面追加文件名，函数名，行号
#define LOG_WARN(msg, ...) MCLogger::getInstance().GetLogger()->warn(SUFFIX(msg), ##__VA_ARGS__)
#define LOG_ERROR(msg, ...) MCLogger::getInstance().GetLogger()->error(SUFFIX(msg), ##__VA_ARGS__)
#define LOG_CRITICAL(msg, ...) MCLogger::getInstance().GetLogger()->critical(SUFFIX(msg), ##__VA_ARGS__)

//控制台输出
#define LOG_TRACE_CL(...) MCLogger::getInstance().GetConsoleLogger()->trace(__VA_ARGS__)
#define LOG_DEBUG_CL(...) MCLogger::getInstance().GetConsoleLogger()->debug(__VA_ARGS__)

#define LOG_INFO_CL(...) MCLogger::getInstance().GetConsoleLogger()->info(__VA_ARGS__)
//以上等同于 #define LOG_INFO_CL(...) spdlog::get("console")->info(__VA_ARGS__)

//在警告和错误级别的日志后面追加文件名，函数名，行号
#define LOG_WARN_CL(msg, ...) MCLogger::getInstance().GetConsoleLogger()->warn(SUFFIX(msg), ##__VA_ARGS__)
#define LOG_ERROR_CL(msg, ...) MCLogger::getInstance().GetConsoleLogger()->error(SUFFIX(msg), ##__VA_ARGS__)
#define LOG_CRITICAL_CL(msg, ...) MCLogger::getInstance().GetConsoleLogger()->critical(SUFFIX(msg), ##__VA_ARGS__)

//文件日志类
class MCLogger
{
public:
    static MCLogger& getInstance();

    // 禁止复制和赋值
    MCLogger(const MCLogger&) = delete;
    MCLogger& operator=(const MCLogger&) = delete;
    //初始化日志器
    void InitLog(const std::string& logger_name = "default", const std::string& log_file_path = "log.txt", int log_level = spdlog::level::trace);

    void EndLog();
    //设置日志等级
    void SetLevel(int level = spdlog::level::trace);

    auto GetLogger()
    {
        return _logger_;
    }

    auto GetConsoleLogger()
    {
        return _loggerConsole_;
    }

private:
    static MCLogger* instance;
    static std::mutex mutex;

    std::shared_ptr<spdlog::logger> _loggerConsole_;

    std::shared_ptr<spdlog::logger> _logger_;
    // 禁止在外部实例化对象
    MCLogger();
    ~MCLogger();
};

#endif
