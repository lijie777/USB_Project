#include "MCLogger.h"

#include <chrono>   // 包含时间头文件
#include <iomanip>  // 包含格式化头文件
#include <sstream>  // 包含字符串流头文件

MCLogger* MCLogger::instance = nullptr;
std::mutex MCLogger::mutex;

MCLogger::MCLogger()
{
    // Initialize the console logger in the constructor
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    _loggerConsole_ = std::make_shared<spdlog::logger>("console", console_sink);  //创建的日志器名称不能重复，否则会引起崩溃
    _loggerConsole_->set_level(spdlog::level::trace);

    //注册日志器,注册了以后可以直接使用spdlog::get("logger_name")来获取日志器，这里的"logger_name"是日志器的名称(如上面的"console")，可以自定义
    spdlog::register_logger(_loggerConsole_);
}

MCLogger::~MCLogger()
{
    EndLog();
}

MCLogger& MCLogger::getInstance()
{
    // 为了避免每次调用getInstance()都需要加锁解锁，使用双重检查锁定
    if (nullptr == instance) {
        std::lock_guard<std::mutex> lock(mutex);

        instance = new MCLogger;
    }
    return *instance;
}

void MCLogger::InitLog(const std::string& logger_name /* = "default"*/, const std::string& log_file_path /* = "log.txt"*/,
                       int log_level /*= spdlog::level::trace*/)
{
    // 获取当前日期和时间
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    // 格式化日期和时间为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d-%H-%M-%S");
    std::string log_file_name = ss.str() + "_log.txt";

    // 拼接日志文件路径
    std::string logFile = log_file_path + "/" + log_file_name;

    // 设置日志格式
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    // 创建文件输出sink，设置日志文件大小限制
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logFile,           // 日志文件路径
                                                                            10 * 1024 * 1024,  // 日志文件大小上限，这里是10MB
                                                                            3);                // 保留的日志文件数量

    /* rotating_file_sink_mt 的第二个参数是最大日志文件大小，这里设置为 10 MB，
    第三个参数是最大日志文件个数，这里设置为 3。当日志文件大小达到 10 MB 时，
    会自动创建一个新的日志文件，同时删除最早的一个日志文件，以保证日志文件的数量不超过 3 个。 */

    // 创建logger
    _logger_ = std::make_shared<spdlog::logger>(logger_name, file_sink);
    _logger_->set_level(spdlog::level::trace);  // 设置logger级别

    spdlog::set_level(spdlog::level::trace);  //控制台的日志级别

    //当遇到错误级别以上的立刻刷新到日志
    _logger_->flush_on(spdlog::level::err);

    //每三秒刷新日志
    spdlog::flush_every(std::chrono::seconds(3));

    //设置默认的日志记录器
    spdlog::set_default_logger(_logger_);
}

void MCLogger::EndLog()
{
    spdlog::shutdown();
}

void MCLogger::SetLevel(int level)
{
    _logger_->set_level(static_cast<spdlog::level::level_enum>(level));
    spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
}
