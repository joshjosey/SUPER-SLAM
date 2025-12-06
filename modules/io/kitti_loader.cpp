#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "vo/camera.hpp"
#include "vo/frame.hpp"
using namespace std;
namespace vo
{

    vector<string> load_image_paths(const string &folder)
    {
        vector<string> files;
        // Use OpenCV to glob all PNG files in the folder
        cv::glob(folder + "*.png", files, false);
        sort(files.begin(), files.end());
        return files;
    }

    Camera load_kitti_calib(const std::string &file_path)
    {
        // Open the file
        ifstream f(file_path);
        if (!f)
        {
            cerr << "Unable to open calibration file at '" << file_path << "'" << endl;
            exit(1);
        }

        std::string line;
        Camera cam;
        // Search for the projection matrices
        while (getline(f, line))
        {
            istringstream ss(line);
            std::string label;
            ss >> label;

            if (label == "P_rect_00:")
            {
                // Load left camera projection matrix
                double p_00[12];
                for (int i = 0; i < 12; i++)
                {
                    ss >> p_00[i];
                }

                cam.fx = p_00[0];
                cam.fy = p_00[5];
                cam.cx = p_00[2];
                cam.cy = p_00[6];
            }
            else if (label == "P_rect_01:")
            {
                // Load right camera projection matrix
                double p_01[12];
                for (int i = 0; i < 12; i++)
                {
                    ss >> p_01[i];
                }

                // baseline = -Tx / fx
                cam.baseline = -p_01[3] / cam.fx;
            }
        }

        // Close the file and return the camera
        f.close();
        return cam;
    }

    std::vector<double> load_timestamps(const std::string &file_path)
    {
        std::vector<double> timestamps;
        std::ifstream f(file_path);
        if (!f)
        {
            std::cerr << "Failed to open timestamps file: " << file_path << std::endl;
            return timestamps;
        }

        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty())
                continue;

            // Parse the HH:MM:SS.ns to get total seconds
            // Find the space after date
            size_t split = line.find(' ');
            if (split == std::string::npos)
                continue;

            std::string time_str = line.substr(split + 1);

            // Extract hours, minutes, seconds based on format
            double h = std::stod(time_str.substr(0, 2));
            double m = std::stod(time_str.substr(3, 2));
            double s = std::stod(time_str.substr(6));

            // Convert to total seconds
            double total_seconds = h * 3600.0 + m * 60.0 + s;
            timestamps.push_back(total_seconds);
        }
        return timestamps;
    }
}