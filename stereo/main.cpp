#include <opencv2/opencv.hpp>
#include <iostream>

const std::string img_directory = "data/img_stereo/";

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage : " << argv[0] << " frame number" << std::endl;
        return 0;
    }

    // Open left/right images
    std::string left_filename(img_directory + "left_"),
        right_filename(img_directory + "right_");
    left_filename += argv[1];
    left_filename += ".png";
    right_filename += argv[1];
    right_filename += ".png";

    cv::Mat left_img = cv::imread(left_filename, 0),
        right_img    = cv::imread(right_filename, 0);

    // Compute disparity map
    cv::StereoSGBM stereo(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
    cv::Mat stereo_img;
    stereo(left_img, right_img, stereo_img);
    cv::Mat display_stereo_img;
    stereo_img.convertTo(display_stereo_img, CV_8U);

    // Display images and wait for a key press
    cv::imshow("left image", left_img);
    cv::imshow("right image", right_img);
    cv::imshow("stereo image", display_stereo_img);

    cv::waitKey();

    return 0;
}

