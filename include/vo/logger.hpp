// File for creating output logs
#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <mutex>
#include <filesystem>

namespace vo
{
    /**
     * @brief Logging to file output
     */
    class Logger
    {
    public:
        /**
         * @brief Construct a logger that writes to a file.
         * @param filename Path to log file
         */
        Logger(const std::string &filepath)
        {
            // Create the parent directories as needed
            std::filesystem::path path_obj(filepath);
            if (path_obj.has_parent_path())
            {
                std::filesystem::create_directories(path_obj.parent_path());
            }
            else
            {
                std::cerr << "Invalid log file path: " << filepath << std::endl;
            }

            // Open the log file in append mode
            log_file.open(filepath, std::ios::out | std::ios::trunc); // overwrite every run
            if (!log_file.is_open())
            {
                std::cerr << "Failed to open log file: " << filepath << std::endl;
            }
        }

        /**
         * @brief Log a message to the logger file
         * @param msg Message string
         */
        void log(const std::string &msg)
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (log_file.is_open())
                log_file << msg << std::endl;
            else
                std::cout << msg << std::endl;
        }

    private:
        std::ofstream log_file;
        std::mutex mtx;
    };

}