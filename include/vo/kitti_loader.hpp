#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "vo/camera.hpp"
#include "vo/frame.hpp"

namespace vo
{
    /**
     * @brief Load image file paths from a folder.
     * @param folder Path to image folder
     * @return Vector of sorted image file paths
     */
    std::vector<std::string> load_image_paths(const std::string &folder);

    /**
     * @brief Load KITTI stereo camera calibration from file.
     * @param file Path to KITTI calibration file
     * @return Camera object with loaded intrinsic parameters
     */
    Camera load_kitti_calib(const std::string &file_path);

    /**
     * @brief Load timestamps from a KITTI timestamp file.
     * @param file Path to timestamp file
     * @return Vector of timestamps in seconds
     */
    std::vector<double> load_timestamps(const std::string &file_path);
}