#include "vo/frame.hpp"

namespace vo
{

    Frame::Frame(int id, const cv::Mat &left, const cv::Mat &right)
        : id(id), left_img(left), right_img(right) {}

}
