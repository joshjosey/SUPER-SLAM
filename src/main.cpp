// argument parsing, pipeline orchestration
#include <iostream>
#include <opencv2/opencv.hpp>
#include "vo/camera.hpp"
#include "vo/frame.hpp"
#include "vo/kitti_loader.hpp"
#include "vo/logger.hpp"
#include "vo/viz.hpp"
#include "vo/feature_tracker.hpp"
#include "vo/feature.hpp"
#include "vo/pnp_solver.hpp"
#include "vo/map.hpp"
#include "vo/filter.hpp"

using namespace std;

int main(int argc, char **argv)
{
    // Ensure the correct number of arguments are provided
    if (argc < 4)
    {
        cerr << "Usage: " << argv[0] << " <data_path_prefix> <drive_date> <drive_number> [optional]<output folder path>" << endl;
        cerr << "Example: " << argv[0] << " /data/ 2011_09_30 0016 ./results" << endl;
        return -1;
    }

    // Parse input arguments
    string data_path = argv[1];
    string drive_date = argv[2];
    string drive_number = argv[3];
    string output_path = (argc > 4 ? argv[4] : "./results");

    // Initialize Loggers
    vo::Logger logger(output_path + "/logs/app.log");
    logger.log("Starting SUPER SLAM...");

    // vo::Logger stereo_logger(output_path + "/logs/stereo_matches_count.log");
    // vo::Logger temporal_logger(output_path + "/logs/temporal_matches_count.log");
    vo::Logger pose_logger_raw(output_path + "/logs/pose_estimation_raw.log");
    vo::Logger pose_logger_smoothed(output_path + "/logs/pose_estimation_smoothed.log");

    // Build file paths
    string path_left = data_path + drive_date + "_drive_" + drive_number + "_sync" + "/image_00/data/";
    string path_right = data_path + drive_date + "_drive_" + drive_number + "_sync" + "/image_01/data/";
    string calib_file = data_path + "/calib_cam_to_cam.txt";

    // Load and log camera calibration data
    logger.log("Loading calibration from " + calib_file);
    vo::Camera cam = vo::load_kitti_calib(calib_file);
    logger.log("Loaded calibration" + calib_file);

    // Create a feature tracker, pnp solver, visualizer
    vo::FeatureTracker tracker;
    vo::PnPSolver pnp_solver;
    vo::Visualizer viz(10);

    // Load images paths
    vector<string> left_image_paths = vo::load_image_paths(path_left);
    vector<string> right_image_paths = vo::load_image_paths(path_right);

    if (left_image_paths.empty() || right_image_paths.empty())
    {
        cerr << "Failed to load image paths!" << " at " << path_left << " or " << path_right << endl;
        return -1;
    }
    else
    {
        logger.log("\nLoaded " + to_string(left_image_paths.size()) + " left images from " + path_left);
        logger.log("Loaded " + to_string(right_image_paths.size()) + " right images from " + path_right);
    }

    // Process each stereo image pair as a frame
    vector<vo::Frame> frames;
    vector<double> timestamps = vo::load_timestamps(data_path + drive_date + "_drive_" + drive_number + "_sync/oxts/timestamps.txt");
    logger.log("\nBuilding frames from image pairs...");
    for (size_t i = 0; i < left_image_paths.size() && i < right_image_paths.size(); ++i)
    {
        // Load images
        cv::Mat left_img = cv::imread(left_image_paths[i], cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(right_image_paths[i], cv::IMREAD_GRAYSCALE);
        // If the images could not be loaded, log a warning and skip
        if (left_img.empty() || right_img.empty())
        {
            logger.log("Warning: Could not load image pair at index " + to_string(i));
            continue;
        }
        // Push back a Frame object
        frames.emplace_back(i, left_img, right_img, timestamps[i]);
    }
    logger.log("Built frames: " + to_string(frames.size()));

    // Perform feature matching across stereo and temporal pairs
    logger.log("\nInitializing first frame...");
    if (!frames.empty())
    {
        logger.log("extracted features");
        tracker.extractFeatures(frames[0]);
        logger.log("extracted stereo");

        tracker.matchStereo(frames[0], cam);
        logger.log("saved pose");

        frames[0].pose_raw = cv::Mat::eye(4, 4, CV_64F);
        frames[0].pose_smoothed = cv::Mat::eye(4, 4, CV_64F);
        frames[0].is_pose_accepted = true;
    }
    vo::KalmanFilter filter(100, 1.0, 0.1);
    filter.initialize(frames[0].pose_raw);
    pose_logger_raw.log("Frame 0: " + to_string(frames[0].pose_raw.at<double>(0, 3)) + ", " + to_string(frames[0].pose_raw.at<double>(1, 3)) + ", " + to_string(frames[0].pose_raw.at<double>(2, 3)));
    pose_logger_smoothed.log("Frame 0: " + to_string(frames[0].pose_smoothed.at<double>(0, 3)) + ", " + to_string(frames[0].pose_smoothed.at<double>(1, 3)) + ", " + to_string(frames[0].pose_smoothed.at<double>(2, 3)));
    logger.log("Initialized first frame successfully");

    logger.log("\nParsing remaining frames...");
    for (size_t i = 1; i < frames.size(); i++)
    {
        // Extract features, match stereo pairs, match temporal pairs
        tracker.extractFeatures(frames[i]);
        tracker.matchStereo(frames[i], cam);
        tracker.matchTemporal(frames[i], frames[i - 1]);

        // Free the image memory
        frames[i - 1].left_img.release();
        frames[i - 1].right_img.release();

        // Estimate pose using PnP
        cv::Mat R, t;
        bool pose_estimation_success = pnp_solver.estimatePose(frames[i - 1], frames[i], cam, R, t);

        if (pose_estimation_success)
        {
            // Construct the 4x4 pose matrix
            cv::Mat relative_T = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(relative_T(cv::Rect(0, 0, 3, 3)));
            t.copyTo(relative_T(cv::Rect(3, 0, 1, 3)));
            frames[i].pose_raw = frames[i - 1].pose_raw * relative_T.inv();
            frames[i].is_pose_accepted = true;
            pose_logger_raw.log("Frame " + to_string(i) + ": " + to_string(frames[i].pose_raw.at<double>(0, 3)) + ", " + to_string(frames[i].pose_raw.at<double>(1, 3)) + ", " + to_string(frames[i].pose_raw.at<double>(2, 3)));
        }
        else
        {
            // If pose estimation failed, set to previous pose
            frames[i].pose_raw = frames[i - 1].pose_raw.clone();
            logger.log("Warning: Pose estimation failed for frame " + to_string(i) + ", setting to previous pose.");
        }

        // Calculate dt
        double dt = 0.1;
        if (i < timestamps.size())
        {
            dt = timestamps[i] - timestamps[i - 1];
        }

        // Filter the estimated pose
        frames[i].pose_smoothed = filter.process(frames[i].pose_raw, pose_estimation_success, dt);
        pose_logger_smoothed.log("Frame " + to_string(i) + ": " + to_string(frames[i].pose_smoothed.at<double>(0, 3)) + ", " + to_string(frames[i].pose_smoothed.at<double>(1, 3)) + ", " + to_string(frames[i].pose_smoothed.at<double>(2, 3)));
    }
    logger.log("Done matching points across stereo frames");

    // Visualize trajectory
    vector<cv::Point3f> trajectory;
    for (const auto &frame : frames)
    {
        trajectory.push_back(frame.getPosition("smoothed"));
    }
    viz.plotTrajectory2d(trajectory, "Estimated 2D Trajectory");
    logger.log("Application finished successfully");
    return 0;
}