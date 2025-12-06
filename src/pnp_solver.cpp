#include "vo/pnp_solver.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

namespace vo
{
    void PnPSolver::extract3D2DMatches(const Frame &prev_frame, const Frame &cur_frame, std::vector<cv::Point3f> &points_3d, std::vector<cv::Point2f> &points_2d)
    {
        points_3d.clear();
        points_2d.clear();

        for (const auto &match : cur_frame.temporal_matches)
        {
            // Get the 3D point from the previous frame using the query index
            const cv::Point3f &pt_3d = prev_frame.features.points_3d[match.queryIdx];
            if (pt_3d.z > 0)
            {
                points_3d.push_back(pt_3d);
                // Get the corresponding 2D point in the current frame using the train index
                points_2d.push_back(cur_frame.features.left.keypoints[match.trainIdx].pt);
            }
        }
    }

    bool PnPSolver::estimatePose(const Frame &prev_frame, const Frame &cur_frame, const Camera &cam, cv::Mat &R, cv::Mat &t)
    {
        std::vector<cv::Point3f> points_3d;
        std::vector<cv::Point2f> points_2d;

        // Extract 3D-2D point correspondences between previous and current frames
        extract3D2DMatches(prev_frame, cur_frame, points_3d, points_2d);

        // Check if we have enough points for pnp
        if (points_3d.size() < (size_t)min_pnp_pts || points_2d.size() < (size_t)min_pnp_pts)
        {
            return false;
        }

        cv::Mat K = cam.K();
        cv::Mat rvec, tvec;
        cv::Mat inlier_mask;
        bool pose_flag = cv::solvePnPRansac(points_3d, points_2d, cam.K(),
                                            cv::noArray(), rvec, t, false,
                                            100, 8.0, 0.99, inlier_mask);

        // Make sure there are enough inliers, the pose is ok
        if (!pose_flag || inlier_mask.rows < (size_t)(min_pnp_pts/2)) return false;
        // Convert the rotation vector to rotation matrix
        cv::Rodrigues(rvec, R);
        return pose_flag;
    }
}
