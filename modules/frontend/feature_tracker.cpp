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
        frame.features.points_3d.assign(frame.features.left.keypoints.size(), cv::Point3f(0, 0, 0));
        frame.features.track_2d_3d_idx.assign(frame.features.left.keypoints.size(), -1);

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
                // Potentially add filtering on Z here
                float X = (left_kp.pt.x - cam.cx) * Z / cam.fx;
                float Y = (left_kp.pt.y - cam.cy) * Z / cam.fy;
                cv::Point3f pt(X, Y, Z);

                frame.features.points_3d[match.queryIdx] = cv::Point3f(X, Y, Z);
                frame.features.stereo_matches.push_back(match);
                frame.features.track_2d_3d_idx[match.queryIdx] = static_cast<int>(frame.features.points_3d.size()) - 1;
            }
        }
    }

    // Match temporal features between current and previous frames (using left images)
    void FeatureTracker::matchTemporal(Frame &cur_frame, Frame &prev_frame)
    {
        bf_matcher.match(prev_frame.features.left.descriptors, cur_frame.features.left.descriptors, cur_frame.temporal_matches);
    }
}