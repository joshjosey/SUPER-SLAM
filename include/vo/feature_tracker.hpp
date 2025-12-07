#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "vo/frame.hpp"
#include "vo/camera.hpp"

namespace vo
{
    /**
     * @brief Feature Tracker for detecting and matching features in stereo frames.
     */
    class FeatureTracker
    {
    public:
        // Constructors
        /**
         * @brief Construct a Feature Tracker with given parameters.
         * @param orb_features Number of ORB features to detect
         * @param orb_fast_threshold FAST threshold for ORB feature detection, lower values yield more features
         * @param max_match_distance Maximum distance for valid stereo matches in depth (Z) direction
         */
        FeatureTracker(int orb_features = 1000, int orb_fast_threshold = 20, float max_match_distance = 50.0f);

        // Methods
        /**
         * @brief Detect and compute ORB features in an image.
         * @param image Input image
         * @param keypoints Output vector of detected keypoints
         * @param descriptors Output matrix of computed descriptors
         */
        void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

        /**
         * @brief Match descriptors between two images.
         * @param desc1 Descriptors from the first image
         * @param desc2 Descriptors from the second image
         * @param matches Output vector of matches
         */
        void match(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<cv::DMatch> &matches);

        /**
         * @brief Extract features from a stereo frame.
         * @param frame Frame to extract features from
         */
        void extractFeatures(Frame &frame);

        /**
         * @brief Match stereo features within a frame.
         * @param frame Frame containing stereo images
         * @param cam Camera object with intrinsic parameters
         */
        void matchStereo(Frame &frame, Camera &cam);

        /**
         * @brief Match temporal features between current and previous frames.
         * @param cur_frame Current frame
         * @param prev_frame Previous frame
         * @param cam Camera object with intrinsic parameters
         */
        void matchTemporal(Frame &cur_frame, Frame &prev_frame);

    private:
        // Parameters
        int orb_features;
        int orb_fast_threshold;
        float max_match_distance;

        // ORB detector and Matcher
        cv::Ptr<cv::ORB> orb;
        cv::BFMatcher bf_matcher;
    };
}