#include "vo/filter.hpp"
#include <cmath>
#include <iostream>

namespace vo
{
    KalmanFilter::KalmanFilter(double initial_uncertainty, double process_noise, double measurement_noise, double motion_gate_threshold)
        : motion_gate_threshold(motion_gate_threshold)
    {
        // Initialize State (6x1)
        state = cv::Mat::zeros(6, 1, CV_64F);

        // Initialize P (Covariance)
        P = cv::Mat::eye(6, 6, CV_64F) * initial_uncertainty;

        // Initialize Q (Process Noise) - Assume constant velocity between frames, but add slight noise to account for acceleration changes
        Q = cv::Mat::eye(6, 6, CV_64F) * process_noise;

        // Initialize R (Measurement Noise)
        R = cv::Mat::eye(3, 3, CV_64F) * measurement_noise;

        // Initialize C (Measurement Matrix) - map 3x6 state to 3x1 measurement
        // [1 0 0 0 0 0]
        // [0 1 0 0 0 0]
        // [0 0 1 0 0 0]
        C = cv::Mat::zeros(3, 6, CV_64F);
        C.at<double>(0, 0) = 1;
        C.at<double>(1, 1) = 1;
        C.at<double>(2, 2) = 1;

        // Initialize rotation
        current_rot = cv::Mat::eye(3, 3, CV_64F);
    }

    void KalmanFilter::initialize(const cv::Mat &start_pose)
    {
        // Extract translation [x, y, z]
        state.at<double>(0) = start_pose.at<double>(0, 3);
        state.at<double>(1) = start_pose.at<double>(1, 3);
        state.at<double>(2) = start_pose.at<double>(2, 3);

        // Velocity starts at 0
        state.at<double>(3) = 0;
        state.at<double>(4) = 0;
        state.at<double>(5) = 0;

        // Store rotation
        start_pose(cv::Rect(0, 0, 3, 3)).copyTo(current_rot);

        initialized = true;
    }

    void KalmanFilter::predict(double dt)
    {
        // Define A (Transition Matrix) for Constant Velocity
        // [I  I*dt]
        // [0  I   ]
        cv::Mat A = cv::Mat::eye(6, 6, CV_64F);
        A.at<double>(0, 3) = dt;
        A.at<double>(1, 4) = dt;
        A.at<double>(2, 5) = dt;

        // Predict State: x = A * x
        state = A * state;

        // Predict Covariance: P = A * P * A^T + Q
        P = A * P * A.t() + Q;
    }

    void KalmanFilter::update(const cv::Mat &measurement)
    {
        // Calculate Kalman Gain: K = P * C^T * (C * P * C^T + R)^-1
        cv::Mat K = P * C.t() * (C * P * C.t() + R).inv();

        // Update State: x = x + K * (z - C * x)
        cv::Mat z = measurement;    // 3x1
        cv::Mat z_pred = C * state; // 3x1
        cv::Mat innovation = z - z_pred;
        state = state + (K * innovation);

        // Update Covariance: P = (I - K * C) * P
        cv::Mat I = cv::Mat::eye(6, 6, CV_64F);
        P = (I - K * C) * P;
    }

    cv::Mat KalmanFilter::process(const cv::Mat &T_measured, bool is_valid, double dt)
    {
        if (!initialized)
        {
            initialize(T_measured);
            return T_measured;
        }

        // Predict
        predict(dt);

        // Extract translation measurement
        cv::Mat meas_pos = (cv::Mat_<double>(3, 1) << T_measured.at<double>(0, 3),
                            T_measured.at<double>(1, 3),
                            T_measured.at<double>(2, 3));

        // Gate & Update
        if (is_valid && passGate(meas_pos))
        {
            update(meas_pos);
            T_measured(cv::Rect(0, 0, 3, 3)).copyTo(current_rot);
        }
        // Else: Skip update (prediction only)

        // Reconstruct 4x4 Pose from State
        cv::Mat T_final = cv::Mat::eye(4, 4, CV_64F);

        // Save Rotation
        current_rot.copyTo(T_final(cv::Rect(0, 0, 3, 3)));

        // Save Translation (from Filtered State)
        T_final.at<double>(0, 3) = state.at<double>(0);
        T_final.at<double>(1, 3) = state.at<double>(1);
        T_final.at<double>(2, 3) = state.at<double>(2);

        return T_final;
    }

    bool KalmanFilter::passGate(const cv::Mat &measured_pos)
    {
        // Euclidean distance check if measured state is reasonably close to predicted state
        double dx = state.at<double>(0) - measured_pos.at<double>(0);
        double dy = state.at<double>(1) - measured_pos.at<double>(1);
        double dz = state.at<double>(2) - measured_pos.at<double>(2);
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // State should be within a few meters of prediction
        return dist < motion_gate_threshold; 
    }
}