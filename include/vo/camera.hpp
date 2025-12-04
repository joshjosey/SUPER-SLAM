// intrinsics, distortion, pixel <-> ray
#pragma once
#include <string>
#include <opencv2/opencv.hpp>

namespace vo
{

    /**
     * @brief Represents a stereo camera and its intrinsic parameters.
     **/
    class Camera
    {
    public:
        // Camera intrinsic parameters
        double fx, fy, cx, cy, baseline;

        // Constructors
        Camera() = default;
        /**
         * @brief Construct a Camera with given parameters.
         * @param fx Focal length in x
         * @param fy Focal length in y
         * @param cx Principal point x
         * @param cy Principal point y
         * @param baseline Distance between left and right cameras
         */
        Camera(double fx, double fy, double cx, double cy, double baseline);

        // Utility
        /**
         * @brief Return the 3x3 intrinsic camera matrix K
         * @return cv::Mat 3x3 intrinsic matrix
         */
        cv::Mat K() const;
    };

}
