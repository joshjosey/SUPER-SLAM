// PnP, RANSAC wrappers
#pragma once
#include <opencv2/opencv.hpp>
#include "vo/frame.hpp"
#include "vo/camera.hpp"

namespace vo
{

    class PnPSolver
    {
    public:
        /**
         * @brief Construct a PnP solver for pose estimation
         * @param min_pnp_pts The minimum number of points PnP should use to generate valid poses
         * @param useExtrensicGuess Whether to use an initial guess for the pose
         * @param iterations Number of RANSAC iterations
         * @param reprojection_error Reprojection error threshold for RANSAC in pixels
         * @param confidence RANSAC confidence level
         */
        PnPSolver(int min_pnp_pts = 4, bool useExtrensicGuess = false, int iterations = 100, double reprojection_error = 8.0, double confidence = 0.99) 
            : min_pnp_pts(min_pnp_pts), useExtrensicGuess(useExtrensicGuess), iterations(iterations), reprojection_error(reprojection_error), confidence(confidence) {}

        /**
         * @brief Extrcts corresponding 3d points from the previous frame and 2d points from the current frame
         * @param prev_frame The previous frame (i-1)
         * @param cur_frame The current frame (i)
         * @param points_3d Output parameter for storing the 3d corresponding points
         * @param points_2d Output parameter for storing the 2d corresponding points
         */
        void extract3D2DMatches(const Frame &prev_frame, const Frame &cur_frame, std::vector<cv::Point3f> &points_3d, std::vector<cv::Point2f> &points_2d);

        /**
         * @brief Estimated the pose of the vehicle using PnP
         * @param prev_frame The previous frame (i-1)
         * @param cur_frame The current frame (i)
        *  @param cam Camera object contianing the intrinsic matrix K
         * @param R Output parameter for Rotation matrix from prev -> cur
         * @param t Output parameter for translation matrix from prev -> cur
         */
        bool estimatePose(const Frame &prev_frame, const Frame &cur_frame, const Camera &cam, cv::Mat &R, cv::Mat &t);

    private:
        int min_pnp_pts;
        bool useExtrensicGuess;
        int iterations;
        double reprojection_error;
        double confidence;
    };
}