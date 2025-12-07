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

        /**
         * @brief Converts the pose matrix into a string
         * @param mode The raw or smoothed pose
         * @returns A string vector of the pose
         */
        std::string getPoseMatString(std::string mode)
        {
            cv::Mat pose;
            if (mode == "raw")
            {
                pose = pose_raw;
            }
            else
            {
                pose = pose_smoothed;
            }
            std::stringstream ss;
            // Iterate over the first 3 rows and all 4 columns
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 4; ++col)
                {
                    ss << pose.at<double>(row, col);
                    // Add space if it's not the last element
                    if (!(row == 2 && col == 3))
                    {
                        ss << " ";
                    }
                }
            }
            return ss.str();
        }
    };
}
