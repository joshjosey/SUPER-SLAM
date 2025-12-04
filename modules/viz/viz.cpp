#include "vo/viz.hpp"
#include <opencv2/opencv.hpp>

namespace vo
{
    void Visualizer::showImage(const cv::Mat &img, const std::string &win_name) const
    {
        // Show single image
        cv::imshow(win_name, img);
        cv::waitKey(wait_time);
    }

    void Visualizer::showFrame(const Frame &frame, const std::string &win_name) const
    {
        // Combine left and right images side by side then show
        cv::Mat stereo_img;
        cv::hconcat(frame.left_img, frame.right_img, stereo_img);
        cv::imshow(win_name, stereo_img);
        cv::waitKey(wait_time);
    }

}