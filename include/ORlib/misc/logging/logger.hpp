/**
 * @file logger.hpp
 * @author Adin De'Rosier (adin.derosier@oit.edu)
 * @brief Header file for the class that defines a logger.
 * @version 0.2
 * @date 2023-09-27
 *
 * This file defines a logger.
 */
#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <string>
#include <chrono>

using std::string;

namespace OITRC::misc::logging{
    enum LogLevel {
        INFO,
        DEBUG,
        WARNING,
        ERROR,
        CRITICAL
    };

    class Logger {
    public:
        Logger(LogLevel level = INFO) : logLevel(level) {}

        void setLogLevel(LogLevel level) {
            logLevel = level;
        }

        void info(const string& message, const string& caller =
                __builtin_FILE(), const int& line = __builtin_LINE()) {
            if (logLevel <= INFO) {
                log(caller, line, "INFO", message);
            }
        }

        void debug(const string& message, const string& caller =
                __builtin_FILE(), const int& line = __builtin_LINE()) {
            if (logLevel <= DEBUG) {
                log(caller, line, "DEBUG", message);
            }
        }

        void warning(const string& message, const string& caller =
                __builtin_FILE(), const int& line = __builtin_LINE()) {
            if (logLevel <= WARNING) {
                log(caller, line, "WARNING", message);
            }
        }

        void error(const string& message, const string& caller =
                __builtin_FILE(), const int& line = __builtin_LINE()) {
            if (logLevel <= ERROR) {
                log(caller, line, "ERROR", message);
            }
        }

        void critical(const string& message, const string& caller =
                __builtin_FILE(), const int& line = __builtin_LINE()) {
        if (logLevel <= CRITICAL) {
            log(caller, line, "CRITICAL", message);
            }
        }

    private:
        LogLevel logLevel;

        void log(const string& caller, const int& line, const string& level, const string& message) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

            char buffer[80];
            std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", std::localtime(&time));
            string timestamp(buffer);

            std::cout << timestamp << "," << line << " - " << caller << " - " << level << " - " << message << "\n";
        }
    };
} // namespace OITRC::misc::logger

#endif // LOGGER_HPP
