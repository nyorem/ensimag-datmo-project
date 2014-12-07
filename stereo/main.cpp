#include <opencv2/opencv.hpp>
#include "tp_util.hpp"
#include <iostream>

// Camera parameters
#define U0 258
#define V0 156
#define ALPHAU 410
#define ALPHAV 410
#define BCAM 0.22
#define Z0 1.28

#define MAX_DISPARITY 32

// Directory containing the stereo images
const std::string img_directory = "data/img_stereo/";

// Computes the 3D coordinates associated to a disparity map
cv::Vec3f uvTo3D (float u, float v, float d) {
    cv::Vec3f vec;

    vec[0] = ((u - U0) * BCAM) / d - BCAM / 2.0;
    vec[1] = ALPHAU * BCAM / d;
    vec[2] = Z0 - ((v - V0) * ALPHAU * BCAM) / (ALPHAV * d);

    return vec;
}

// Computes the v-disparity map
cv::Mat v_disparity (cv::Mat const& disp,
                     unsigned int disp_max) {
    cv::Mat v_disparity_map(disp.rows, disp_max, CV_8U, 0);

    for (int v = 0; v < disp.rows; ++v) {
        unsigned char *output = v_disparity_map.ptr<unsigned char>(v);
        for (int u = 0; u < disp.cols; ++u) {
            unsigned char d = disp.at<unsigned char>(v, u);
            if (d > 0) {
                output[d]++;
            }
        }
    }

    return v_disparity_map;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage : " << argv[0] << " frame number" << std::endl;
        return 0;
    }

    // Open left/right images
    std::string left_filename(img_directory + "left_" + argv[1] + ".png"),
        right_filename(img_directory + "right_" + argv[1] + ".png");

    cv::Mat left_img = cv::imread(left_filename, 0),
        right_img    = cv::imread(right_filename, 0);

    // 1.Compute disparity map
    cv::StereoSGBM stereo(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
    cv::Mat stereo_img;
    stereo(left_img, right_img, stereo_img);
    cv::Mat display_stereo_img; // for display
    stereo_img.convertTo(display_stereo_img, CV_8U);

    // 2.Road/obstacles segmentation in Cartesian space
    cv::Mat stereo_threshold_img = stereo_img.clone();
    for(int v = 0; v < stereo_img.rows; ++v) {
        const unsigned char *input = stereo_img.ptr<unsigned char>(v);
        unsigned char *output = stereo_threshold_img.ptr<unsigned char>(v);
        for(int u = 0; u < stereo_img.cols; ++u) {
            float d = float(input[u]) / 16;
            if (d > 0) {
                cv::Vec3f coords3D = uvTo3D(u, v, d);
                // TODO: check
                if (coords3D[2] < 0.2 || coords3D[1] > 2.5) {
                    output[u] = 0;
                }
            }
        }
    }
    cv::Mat display_stereo_threshold_img;
    stereo_threshold_img.convertTo(display_stereo_threshold_img, CV_8U);

    // 3.Road/obstacles segmentation in Disparity space
    // TODO: next line segfault
    /* cv::Mat v_disparity_map = v_disparity(stereo_img, MAX_DISPARITY); */

    // TODO: compute theta and Z0

    // Threshold
    /* cv::Mat v_disparity_map_threshold; */
    /* cv::threshold(v_disparity_map, v_disparity_map_threshold, 60, 0, cv::THRESH_TOZERO); */

    // Ransac
    /* std::vector<cv::Point2f> points; */
    /* for (int i = 0; i < v_disparity_map_threshold.rows; ++i) { */
    /*     for (int j = 0; j < v_disparity_map_threshold.cols; ++j) { */
    /*         points.push_back(cv::Point2f(i, j)); */
    /*     } */
    /* } */
    /* cv::Vec4f line; */
    /* fitLineRansac(points, line); */

    // TODO: filter disparity image

    // 4. Clustering
    // TODO

    // Display images and wait for a key press
    cv::imshow("left image", left_img);
    cv::imshow("right image", right_img);
    cv::imshow("stereo image", display_stereo_img);
    cv::imshow("stereo image segmented", display_stereo_threshold_img);

    cv::waitKey();

    return 0;
}

