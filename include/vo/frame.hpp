// image, timestamp, features for stereo visual odometry
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "vo/feature.hpp"

namespace vo
{

    /**
     * @brief Represents a stereo frame (left + right images).
     */
    class Frame
    {
    public:
        // Frame Data
        int id;
        cv::Mat left_img;
        cv::Mat right_img;
        StereoFeatures features;
        std::vector<cv::DMatch> temporal_matches;

        // Constructor
        /**
         * @brief Construct a stereo frame with images.
         * @param id Frame index
         * @param left Left image (grayscale)
         * @param right Right image (grayscale)
         */
        Frame(int id, const cv::Mat &left, const cv::Mat &right);
    };

}
