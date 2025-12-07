#pragma once
#include <opencv2/opencv.hpp>

namespace vo
{
    class KalmanFilter
    {
    public:
        /**
         * @brief Construct a Kalman Filter with standard matrices.
         * State vector is 6x1: [x, y, z, vx, vy, vz]
         */
        KalmanFilter(double initial_uncertainty = 100, double process_noise = 0.01, double measurement_noise = 0.1, double motion_gate_threshold = 2.0);

        /**
         * @brief Initialize the filter state.
         * @param start_pose The initial 4x4 pose matrix.
         */
        void initialize(const cv::Mat &start_pose);

        /**
         * @brief Wrapper function that orchestrates Predict -> Gate -> Update.
         * Returns the full 4x4 smoothed pose.
         */
        cv::Mat process(const cv::Mat &T_measured, bool is_valid, double dt);

        // --- Math Functions (Inspired by Python) ---

        /**
         * @brief Predict Step: x = A*x, P = A*P*A' + Q
         * @param dt Delta time since last frame
         */
        void predict(double dt);

        /**
         * @brief Update Step: K = P*C'*(C*P*C' + R)^-1, x = x + K*(z - Cx), P = (I - K*C)*P
         * @param measurement 3x1 vector (x, y, z) from PnP
         */
        void update(const cv::Mat &measurement);

        /**
         * @brief Motion Gating to check if the measured position is within reasonable bounds of the predicted position
         * @param measured_pos 3x1 vector (x, y, z) from PnP
         * @return true if within gate, false otherwise
         */ 
        bool passGate(const cv::Mat &measured_pos);

    private:
        double motion_gate_threshold; // Threshold for motion gating
        // State
        bool initialized = false;
        cv::Mat state; // 6x1 vector [x, y, z, vx, vy, vz]
        cv::Mat P;     // 6x6 Error Covariance Matrix

        // System Matrices
        cv::Mat Q; // 6x6 Process Noise Covariance
        cv::Mat R; // 3x3 Measurement Noise Covariance
        cv::Mat C; // 3x6 Measurement Matrix

        // Helper to store the latest rotation (we only filter translation)
        cv::Mat current_rot;
    };
}