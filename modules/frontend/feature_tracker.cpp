#include "vo/feature_tracker.hpp"
#include "vo/frame.hpp"
#include "vo/camera.hpp"
using namespace std;

namespace vo
{
    // Constructor
    FeatureTracker::FeatureTracker(int orb_features, int orb_fast_threshold) : orb_features(orb_features), orb_fast_threshold(orb_fast_threshold)
    {
        orb = cv::ORB::create(orb_features, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, orb_fast_threshold);
        bf_matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
    }

    // Detect and compute ORB features in an image
    void FeatureTracker::detectAndCompute(const cv::Mat &image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
    {
        orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
    }

    // Match descriptors between two images
    void FeatureTracker::match(const cv::Mat &desc1, const cv::Mat &desc2, vector<cv::DMatch> &matches)
    {
        bf_matcher.match(desc1, desc2, matches);
    }

    // Extract features from a stereo frame
    void FeatureTracker::extractFeatures(Frame &frame)
    {
        detectAndCompute(frame.left_img, frame.features.left.keypoints, frame.features.left.descriptors);
        detectAndCompute(frame.right_img, frame.features.right.keypoints, frame.features.right.descriptors);
    }

    // Match stereo features within a frame
    void FeatureTracker::matchStereo(Frame &frame, Camera &cam)
    {
        vector<cv::DMatch> init_matches;
        bf_matcher.match(frame.features.left.descriptors, frame.features.right.descriptors, init_matches);

        // Stereo matches will deal with 3d point computation, filter out matches that cannot provide depth information
        for (auto &match : init_matches)
        {
            const auto &left_kp = frame.features.left.keypoints[match.queryIdx];
            const auto &right_kp = frame.features.right.keypoints[match.trainIdx];

            // If there is not enough horizontal motion, the match provides not depth information
            float disparity = left_kp.pt.x - right_kp.pt.x;
            if (disparity > 0)
            {
                float Z = static_cast<float>(cam.fx * cam.baseline / disparity);
                float X = (left_kp.pt.x - cam.cx) * Z / cam.fx;
                float Y = (left_kp.pt.y - cam.cy) * Z / cam.fy;
                cv::Point3f pt(X, Y, Z);
                frame.features.points_3d.push_back(pt);
                frame.features.stereo_matches.push_back(match);
            }
        }
    }

    // Match temporal features between current and previous frames (using left images)
    void FeatureTracker::matchTemporal(Frame &cur_frame, Frame &prev_frame, Camera &cam)
    {
        bf_matcher.match(prev_frame.features.left.descriptors, cur_frame.features.left.descriptors, cur_frame.temporal_matches);

        // Extract matched points for essential matrix computation
        std::vector<cv::Point2f> prev_match_pts, cur_match_pts;
        for (const auto &m : cur_frame.temporal_matches)
        {
            prev_match_pts.push_back(prev_frame.features.left.keypoints[m.queryIdx].pt);
            cur_match_pts.push_back(cur_frame.features.left.keypoints[m.trainIdx].pt);
        }

        // Find essential matrix and filter matches using RANSAC
        std::vector<uchar> inlier_mask;
        cv::Mat E = cv::findEssentialMat(prev_match_pts, cur_match_pts, cam.fx, cv::Point2d(cam.cx, cam.cy), cv::RANSAC, 0.99, 1.0, inlier_mask);
        std::vector<cv::DMatch> filtered_matches;
        for (size_t i = 0; i < cur_frame.temporal_matches.size(); ++i)
        {
            if (inlier_mask[i])
                filtered_matches.push_back(cur_frame.temporal_matches[i]);
        }
        cur_frame.temporal_matches = filtered_matches;
    }
}