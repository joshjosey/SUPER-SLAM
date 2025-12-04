#include "vo/viz.hpp"
#include <opencv2/opencv.hpp>
#include <vo/frame.hpp>
using namespace std;
namespace vo
{
    void Visualizer::showImage(const cv::Mat &img, const string &win_name) const
    {
        // Show single image
        cv::imshow(win_name, img);
        cv::waitKey(wait_time);
    }

    void Visualizer::showFrame(const Frame &frame, const string &win_name) const
    {
        // Combine left and right images side by side then show
        cv::Mat stereo_img;
        cv::hconcat(frame.left_img, frame.right_img, stereo_img);
        cv::imshow(win_name, stereo_img);
        cv::waitKey(wait_time);
    }

    void Visualizer::showStereoMatches(const Frame &frame, const string &win_name) const
    {
        // Extract the matches from the struct
        // Draw matches between left and right images
        cv::Mat match_img;
        cv::drawMatches(frame.left_img, frame.features.left.keypoints,
                        frame.right_img, frame.features.right.keypoints,
                        frame.features.stereo_matches, match_img);
        cv::imshow(win_name, match_img);
        cv::waitKey(wait_time);
    }

    void Visualizer::showTemporalMatches(const Frame &cur_frame, const Frame &prev_frame, const std::string &win_name) const
    {
        // Draw matches between current and previous frame using left images
        cv::Mat match_img;
        cv::drawMatches(prev_frame.left_img, prev_frame.features.left.keypoints,
                        cur_frame.left_img, cur_frame.features.left.keypoints,
                        cur_frame.temporal_matches, match_img);
        cv::imshow(win_name, match_img);
        cv::waitKey(wait_time);
    }
    void Visualizer::showTemporalMatches(const cv::Mat &cur_image, const cv::Mat &prev_image, const std::vector<cv::KeyPoint> &cur_kp, const std::vector<cv::KeyPoint> &prev_kp, std::vector<cv::DMatch> matches, const std::string &win_name) const
    {
        // Draw matches between current and previous images
        cv::Mat match_img;
        cv::drawMatches(prev_image, prev_kp,
                        cur_image, cur_kp,
                        matches, match_img);
        cv::imshow(win_name, match_img);
        cv::waitKey(wait_time);
    }
}