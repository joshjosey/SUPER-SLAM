// image, timestamp, features for stereo visual odometry
#pragma once
#include <opencv2/opencv.hpp>
#include <string>

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
