#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace vo
{
    struct Config
    {
        // Dataset
        std::string data_path;
        std::string drive_date;
        std::string drive_number;
        std::string output_path;

        // Tracker
        int orb_features;
        int orb_fast_thresh;
        double max_depth;

        // PnP
        int pnp_min_inliers;
        bool pnp_use_guess;
        int pnp_iterations;
        double pnp_reproj_error;
        double pnp_confidence;

        // Filter
        double kf_init_uncertainty;
        double kf_process_noise;
        double kf_measure_noise;
        double kf_motion_gate;

        static Config load(const std::string& path)
        {
            Config c;
            try {
                YAML::Node config = YAML::LoadFile(path);

                // Dataset
                c.data_path = config["dataset"]["root_path"].as<std::string>();
                c.drive_date = config["dataset"]["drive_date"].as<std::string>();
                c.drive_number = config["dataset"]["drive_number"].as<std::string>();
                c.output_path = config["dataset"]["output_path"].as<std::string>();

                // Tracker
                c.orb_features = config["feature_tracker"]["num_features"].as<int>();
                c.orb_fast_thresh = config["feature_tracker"]["fast_threshold"].as<int>();
                c.max_depth = config["feature_tracker"]["max_depth"].as<double>();

                // PnP
                c.pnp_min_inliers = config["pnp_solver"]["min_inliers"].as<int>();
                c.pnp_use_guess = config["pnp_solver"]["use_extrinsic_guess"].as<bool>();
                c.pnp_iterations = config["pnp_solver"]["ransac_iterations"].as<int>();
                c.pnp_reproj_error = config["pnp_solver"]["reprojection_error"].as<double>();
                c.pnp_confidence = config["pnp_solver"]["confidence"].as<double>();

                // Filter
                c.kf_init_uncertainty = config["kalman_filter"]["initial_uncertainty"].as<double>();
                c.kf_process_noise = config["kalman_filter"]["process_noise"].as<double>();
                c.kf_measure_noise = config["kalman_filter"]["measurement_noise"].as<double>();
                c.kf_motion_gate = config["kalman_filter"]["motion_gate_dist"].as<double>();
            } 
            catch (const YAML::Exception& e) {
                std::cerr << "Error loading config file: " << e.what() << std::endl;
                exit(-1);
            }
            return c;
        }
    };
}