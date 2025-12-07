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
#include "vo/filter.hpp"
#include "vo/config.hpp"

using namespace std;

int main(int argc, char **argv)
{
    // Take the path to the config as an input argument
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <path_to_config.yaml>" << endl;
        cerr << "Example: " << argv[0] << " configs/kitti_0016.yaml" << endl;
        return -1;
    }

    // Load Configuration
    string config_path = argv[1];
    vo::Config cfg = vo::Config::load(config_path);

    // Initialize Loggers
    string log_path = cfg.drive_date + "_drive_" + cfg.drive_number + "_poses";
    vo::Logger logger(cfg.output_path + "/logs/app.log");
    vo::Logger pose_logger_raw(cfg.output_path + "/logs/" + log_path + "_raw.txt");
    vo::Logger pose_logger_smoothed(cfg.output_path + "/logs/" + log_path + ".txt");
    logger.log("Starting SUPER SLAM...");
    logger.log("Data Path: " + cfg.data_path);
    logger.log("Drive Date: " + cfg.drive_date);
    logger.log("Drive Number: " + cfg.drive_number);
    logger.log("Output Path: " + cfg.output_path);

    // Build file paths
    string path_left = cfg.data_path + cfg.drive_date + "_drive_" + cfg.drive_number + "_sync" + "/image_00/data/";
    string path_right = cfg.data_path + cfg.drive_date + "_drive_" + cfg.drive_number + "_sync" + "/image_01/data/";
    string calib_file = cfg.data_path + "calib_cam_to_cam.txt";

    // Load and log camera calibration data
    vo::Camera cam = vo::load_kitti_calib(calib_file);
    logger.log("Loaded calibration from " + calib_file);

    // Create a feature tracker, pnp solver, kalman filter
    vo::FeatureTracker tracker(cfg.orb_features, cfg.orb_fast_thresh);
    vo::PnPSolver pnp_solver(cfg.pnp_min_inliers, cfg.pnp_use_guess, cfg.pnp_iterations, cfg.pnp_reproj_error, cfg.pnp_confidence);
    vo::KalmanFilter filter(cfg.kf_init_uncertainty, cfg.kf_process_noise, cfg.kf_measure_noise);

    // Load images paths
    vector<string> left_image_paths = vo::load_image_paths(path_left);
    vector<string> right_image_paths = vo::load_image_paths(path_right);

    if (left_image_paths.empty() || right_image_paths.empty())
    {
        logger.log("Failed to load image paths! at " + path_left + " or " + path_right);
        return -1;
    }
    logger.log("Loaded " + to_string(left_image_paths.size()) + " left image paths from " + path_left);
    logger.log("Loaded " + to_string(right_image_paths.size()) + " right image paths from " + path_right);

    // Load timestamps
    vector<double> timestamps = vo::load_timestamps(cfg.data_path + cfg.drive_date + "_drive_" + cfg.drive_number + "_sync/oxts/timestamps.txt");
    if (timestamps.empty())
    {
        logger.log("Failed to load timestamps!");
        return -1;
    }
    logger.log("Loaded " + to_string(timestamps.size()) + " timestamps");

    // Process each stereo image pair as a frame
    vector<vo::Frame> frames;
    frames.reserve(left_image_paths.size());
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
        tracker.extractFeatures(frames[0]);
        tracker.matchStereo(frames[0], cam);
        frames[0].pose_raw = cv::Mat::eye(4, 4, CV_64F);
        frames[0].pose_smoothed = cv::Mat::eye(4, 4, CV_64F);
        frames[0].is_pose_accepted = true;
    }

    // Initialize the Kalman Filter with the first frame pose
    filter.initialize(frames[0].pose_raw);
    pose_logger_raw.log(frames[0].getPoseMatString("raw"));
    pose_logger_smoothed.log(frames[0].getPoseMatString("smoothed"));

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

            // Chain with previous pose
            frames[i].pose_raw = frames[i - 1].pose_raw * relative_T.inv();
            frames[i].is_pose_accepted = true;
            pose_logger_raw.log(frames[i].getPoseMatString("raw"));
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
            dt = timestamps[i] - timestamps[i - 1];

        // Filter the estimated pose
        frames[i].pose_smoothed = filter.process(frames[i].pose_raw, pose_estimation_success, dt);
        pose_logger_smoothed.log(frames[i].getPoseMatString("smoothed"));
    }
    logger.log("Completed pose estimation for all frames.");

    // Visualize trajectory
    vo::Visualizer viz;
    vector<cv::Point3f> trajectory;
    for (const auto &frame : frames)
    {
        trajectory.push_back(frame.getPosition("smoothed"));
    }
    viz.plotTrajectory2d(trajectory, "Estimated 2D Trajectory", cfg.output_path + "/plots/" + cfg.drive_date + "_" + cfg.drive_number + "_trajectory.png");
    logger.log("Application finished successfully");
    return 0;
}