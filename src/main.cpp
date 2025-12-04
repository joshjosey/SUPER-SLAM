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

using namespace std;

int main(int argc, char **argv)
{
    // Ensure the correct number of arguments are provided
    if (argc < 4)
    {
        cerr << "Usage: " << argv[0] << " <path_to_data_and_calib> <drive_date> <drive_number> [optional]<output folder path>" << endl;
        cerr << "Example: " << argv[0] << " /path/to/kitti/2011_09_26/ 0001 /path/to/output" << endl;
        return -1;
    }

    // Parse input arguments
    string output_path = "./output";
    string data_path = argv[1];
    string drive_date = argv[2];
    string drive_number = argv[3];
    if (argc > 4)
    {
        output_path = argv[4];
    }

    // Build file paths
    string path_left = data_path + drive_date + "_drive_" + drive_number + "_sync" + "/image_00/data/";
    string path_right = data_path + drive_date + "_drive_" + drive_number + "_sync" + "/image_01/data/";
    string calib_file = data_path + "/calib_cam_to_cam.txt";

    // Initialize Helper Objects
    vo::Visualizer viz(10);
    vo::Logger logger(output_path + "/logs/app.log");
    vo::Logger stereo_logger(output_path + "/logs/stereo_matches_count.log");
    vo::Logger temporal_logger(output_path + "/logs/temporal_matches_count.log");
    logger.log("Starting application...");

    // Load and log camera calibration data
    vo::Camera cam = vo::load_kitti_calib(calib_file);
    logger.log("Loaded calibration from " + calib_file);
    logger.log("fx = " + to_string(cam.fx) + "  fy = " + to_string(cam.fy));
    logger.log("cx = " + to_string(cam.cx) + "  cy = " + to_string(cam.cy));
    logger.log("baseline = " + to_string(cam.baseline));

    // Create a feature tracker
    vo::FeatureTracker tracker;
    logger.log("\nInitialized Feature Tracker");

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
    logger.log("\nBuilding frames from image pairs...");
    for (size_t i = 0; i < left_image_paths.size() && i < right_image_paths.size(); ++i)
    {
        // Load images
        cv::Mat left_img = cv::imread(left_image_paths[i], cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(right_image_paths[i], cv::IMREAD_GRAYSCALE);

        // If the images could not be loaded, log a warning and skip
        if (left_img.empty() || right_img.empty())
        {
            cout << "Warning: Could not load image pair at index " << i << endl;
            logger.log("Warning: Could not load image pair at index " + to_string(i));
            continue;
        }

        // Create a Frame object
        vo::Frame frame(i, left_img, right_img);
        frames.push_back(frame);
    }
    logger.log("Finished building " + to_string(frames.size()) + " frames");

    // Perform feature matching across stereo and temporal pairs
    logger.log("\nMatching points across stereo frame pairs...");

    tracker.extractFeatures(frames[0]);
    tracker.matchStereo(frames[0], cam);
    stereo_logger.log("Found " + to_string(frames[0].features.stereo_matches.size()) + " stereo matches for frame 0");
    for (size_t i = 1; i < frames.size(); i++)
    {
        // Extract features and match stereo pairs
        tracker.extractFeatures(frames[i]);
        tracker.matchStereo(frames[i], cam);
        stereo_logger.log("Frame " + to_string(i) + ": " + to_string(frames[i].features.stereo_matches.size()));

        // Match temporal features with previous frame
        tracker.matchTemporal(frames[i], frames[i - 1], cam);
        temporal_logger.log("Frame " + to_string(i) + "-" + to_string(i - 1) + ": " + to_string(frames[i].temporal_matches.size()));
        // viz.showTemporalMatches(frames[i].left_img, frames[i - 1].left_img, frames[i].features.left.keypoints, frames[i - 1].features.left.keypoints, frames[i].temporal_matches);
        // viz.showStereoMatches(frames[i]);
    }
    logger.log("Done matching points across stereo frames");

    logger.log("Application finished successfully");
    return 0;
}