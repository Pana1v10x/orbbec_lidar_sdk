#include "logger.hpp"

#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <iomanip>
#include <sstream>
namespace ob_lidar {

const spdlog::details::registry& Logger::registry_ =
    spdlog::details::registry::instance();
// default
static constexpr auto DEFAULT_CONSOLE_LOG_LEVEL = LogLevel::INFO;
static constexpr auto DEFAULT_FILE_LOG_LEVEL = LogLevel::INFO;
static constexpr bool LOG_TO_CONSOLE = true;
static constexpr bool LOG_TO_CALLBACK = true;
static constexpr bool LOG_TO_FILE = true;
static constexpr size_t DEFAULT_MAX_FILE_SIZE_MB = 10;
static constexpr size_t DEFAULT_MAX_FILE_NUM = 10;
const static std::string DEFAULT_LOG_FILE_DIR = "./logs";
const static std::string DEFAULT_LOG_FILE_PATTERN =
    "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%g:%#] %v";

spdlog::level::level_enum LogLevelToSpdlogLevel(LogLevel log_level) {
    switch (log_level) {
        case LogLevel::TRACE:
            return spdlog::level::trace;
        case LogLevel::DEBUG:
            return spdlog::level::debug;
        case LogLevel::INFO:
            return spdlog::level::info;
        case LogLevel::WARNING:
            return spdlog::level::warn;
        case LogLevel::ERR:
            return spdlog::level::err;
        case LogLevel::CRITICAL:
            return spdlog::level::critical;
        default:
            return spdlog::level::info;
    }
}

Logger::~Logger() {
    std::cout << "Logger destroyed" << std::endl;
    spdlog::shutdown();
}

std::tm safeLocaltime(const std::time_t& time) {
    std::tm result{};
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);

    std::tm* local_time = std::localtime(&time);
    if (local_time != nullptr) {
        result = *local_time;
    }
    return result;
}

void Logger::logCallback(const spdlog::details::log_msg& msg) const {
    if (callback_) {
        LogMessage log_msg;
        log_msg.level = static_cast<LogLevel>(msg.level);
        log_msg.message = std::string(msg.payload.data(), msg.payload.size());
        callback_(log_msg);
    }
}

void Logger::setLogCallback(const logMessageCallback& callback) {
    callback_ = callback;
}

Logger::Logger(const std::shared_ptr<LoggerConfig> config) {
    spdlog::level::level_enum console_log_level =
        LogLevelToSpdlogLevel(DEFAULT_CONSOLE_LOG_LEVEL);
    spdlog::level::level_enum file_log_level =
        LogLevelToSpdlogLevel(DEFAULT_FILE_LOG_LEVEL);
    bool log_to_console = LOG_TO_CONSOLE;
    bool log_to_file = LOG_TO_FILE;
    bool log_to_callback = LOG_TO_CALLBACK;
    size_t max_file_size_mb = DEFAULT_MAX_FILE_SIZE_MB;
    size_t max_file_num = DEFAULT_MAX_FILE_NUM;
    std::string log_file_dir = DEFAULT_LOG_FILE_DIR;
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = safeLocaltime(now_time_t);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
    std::string log_file_name = "ob_lidar_" + oss.str();
    std::string logger_name = "ob_lidar";

    if (config != nullptr) {
        console_log_level = LogLevelToSpdlogLevel(config->getConsoleLogLevel());
        file_log_level = LogLevelToSpdlogLevel(config->getFileLogLevel());
        log_to_console = config->shouldLogToConsole();
        log_to_file = config->shouldLogToFile();
        log_to_callback = config->shouldLogToCallback();
        max_file_size_mb = config->getMaxLogFileSize();
        max_file_num = config->getMaxLogFileNum();
        log_file_dir = config->getLogFileDir();
    }
    std::vector<spdlog::sink_ptr> sinks;

    if (log_to_console) {
        // create console sink
        console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink_->set_level(console_log_level);
        console_sink_->set_pattern(DEFAULT_LOG_FILE_PATTERN);
        sinks.emplace_back(console_sink_);
    }
    if (log_to_file) {
        // create file sink

        file_sink_ = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file_dir + "/" + log_file_name + ".log",
            max_file_size_mb * 1024 * 1024, max_file_num);
        file_sink_->set_level(file_log_level);
        file_sink_->set_pattern(DEFAULT_LOG_FILE_PATTERN);
        sinks.emplace_back(file_sink_);
    }

    if (log_to_callback) {
        callback_sink_ = std::make_shared<spdlog::sinks::callback_sink_mt>(
            [this](const spdlog::details::log_msg& msg) { logCallback(msg); });
        callback_sink_->set_level(console_log_level);
        callback_sink_->set_pattern(DEFAULT_LOG_FILE_PATTERN);
        sinks.emplace_back(callback_sink_);
    }

    // create logger
    std::shared_ptr<spdlog::logger> logger;
    if (config && config->enableAsync()) {
        spdlog::init_thread_pool(1024, 4);
        logger = std::make_shared<spdlog::async_logger>(
            logger_name, sinks.begin(), sinks.end(), spdlog::thread_pool(),
            spdlog::async_overflow_policy::block);
        spdlog::flush_every(std::chrono::seconds(1));
    } else {
        logger = std::make_shared<spdlog::logger>(logger_name, sinks.begin(),
                                                  sinks.end());
    }
    logger->set_level(spdlog::level::trace);
    set_default_logger(logger);
    spdlog::flush_on(spdlog::level::trace);
}
}  // namespace ob_lidar
