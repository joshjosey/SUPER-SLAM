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
        double timestamp;
        StereoFeatures features;
        std::vector<cv::DMatch> temporal_matches;
        cv::Mat pose_raw = cv::Mat::eye(4, 4, CV_64F);      // 4x4 transformation matrix (Estimated by PnP)
        cv::Mat pose_smoothed = cv::Mat::eye(4, 4, CV_64F); // 4x4 transformation matrix (Smoothed by Kalman)
        bool is_pose_accepted = false;

        // Constructor
        /**
         * @brief Construct a stereo frame with images.
         * @param id Frame index
         * @param left Left image (grayscale)
         * @param right Right image (grayscale)
         * @param timestamp Timestamp of the frame
         */
        Frame(int id, const cv::Mat &left, const cv::Mat &right, double timestamp = 0.0)
            : id(id), left_img(left), right_img(right), timestamp(timestamp) {
              };

        // Helper to get global position
        cv::Point3f getPosition(std::string mode) const
        {
            if (mode == "raw")
            {
                return cv::Point3f(pose_raw.at<double>(0, 3), pose_raw.at<double>(1, 3), pose_raw.at<double>(2, 3));
            }
            else
            {
                return cv::Point3f(pose_smoothed.at<double>(0, 3), pose_smoothed.at<double>(1, 3), pose_smoothed.at<double>(2, 3));
            }
        }
    };

}
