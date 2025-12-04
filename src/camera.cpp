#include "vo/camera.hpp"

namespace vo
{
    Camera::Camera(double fx, double fy, double cx, double cy, double baseline)
        : fx(fx), fy(fy), cx(cx), cy(cy), baseline(baseline) {}

    cv::Mat Camera::K() const
    {
        cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                     0, fy, cy,
                     0, 0, 1);
        return K;
    }

}