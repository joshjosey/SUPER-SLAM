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
        cv::Mat pose = cv::Mat::eye(4, 4, CV_64F); // 4x4 transformation matrix
        bool pose_accepted = false;

        // Constructor
        /**
         * @brief Construct a stereo frame with images.
         * @param id Frame index
         * @param left Left image (grayscale)
         * @param right Right image (grayscale)
         */
        Frame(int id, const cv::Mat &left, const cv::Mat &right);

        // Helper to get global position
        cv::Point3f getPosition() const
        {
            return cv::Point3f(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
        }
    };

}
