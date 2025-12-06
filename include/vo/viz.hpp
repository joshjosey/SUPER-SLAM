// Visualization functions
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "vo/frame.hpp"

namespace vo
{

    /**
     * @brief Visualizer class for displaying frames.
     */
    class Visualizer
    {
    public:
        // Constructor
        /**
         * @brief Construct a Visualizer.
         * @param wait_time Time to wait between frames in milliseconds (Optional, default=10)
         */
        Visualizer(int wait_time = 10) : wait_time(wait_time) {}

        /**
         * @brief Update wait time between frames.
         * @param time Time in milliseconds to wait between frames
         */
        void setWaitTime(int time) { wait_time = time; }

        /**
         * @brief Show a single image
         * @param img Image to display
         * @param win_name Window name (Optional)
         */
        void showImage(const cv::Mat &img, const std::string &win_name = "Image") const;

        /**
         * @brief Show stereo frame in one combined window.
         * @param frame Frame to display
         * @param win_name Window name (Optional)
         */
        void showFrame(const Frame &frame, const std::string &win_name = "Frame") const;

        /**
         * @brief Show stereo matches
         * @param frame Frame to display
         * @param win_name Window name (Optional)
         */
        void showStereoMatches(const Frame &frame, const std::string &win_name = "Stereo Matches") const;

        /**
         * @brief Show temporal matches between current and previous frames. (Left images)
         * @param cur_frame Current frame
         * @param prev_frame Previous frame
         * @param win_name Window name (Optional)
         */
        void showTemporalMatches(const Frame &cur_frame, const Frame &prev_frame, const std::string &win_name = "Temporal Matches") const;
        /**
         * @brief Show temporal matches between current and previous frames. (Choose either left or right images)
         * @param cur_image Current image
         * @param prev_image Previous image
         * @param cur_kp Keypoints in current image
         * @param prev_kp Keypoints in previous image
         * @param matches Matches between current and previous images
         * @param win_name Window name (Optional)
         */
        void showTemporalMatches(const cv::Mat &cur_image, const cv::Mat &prev_image, const std::vector<cv::KeyPoint> &cur_kp, const std::vector<cv::KeyPoint> &prev_kp, std::vector<cv::DMatch> matches, const std::string &win_name = "Temporal Matches") const;

        /**
         * @brief Plot the trajectory in 2d (XZ frame) using matplotlibcpp
         * @param positions 3d positions
         * @param title Plot title (optinal)
         */
        void plotTrajectory2d(const std::vector<cv::Point3f> &positions, const std::string &title = "2D Plot") const;

    private:
        // pause duration for cv::imshow
        int wait_time;
    };
}
