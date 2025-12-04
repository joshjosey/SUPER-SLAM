// keypoints, descriptors, tracks
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "vo/frame.hpp"
#include "vo/camera.hpp"

namespace vo
{
    /**
     * @brief Represents features detected in an image as keypoints and descriptors.
     */
    class Features
    {
    public:
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
    };

    /**
     * @brief Represents stereo features in a frame including left and right image features, stereo matches, and 3D points.
     */
    class StereoFeatures
    {
    public:
        Features left;
        Features right;
        std::vector<cv::DMatch> stereo_matches;
        std::vector<cv::Point3f> points_3d;
    };
}