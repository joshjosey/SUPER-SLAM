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
}