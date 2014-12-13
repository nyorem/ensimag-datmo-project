#include <opencv2/opencv.hpp>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>

#define drawCross( center, color, d )                                 \
    cv::line(display_grid_large, cv::Point( center.x - d, center.y - d ), cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
    cv::line(display_grid_large, cv::Point( center.x + d, center.y - d ), cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

const std::string img_directory = "data/img/";

cv::Point2f centerBox (float x_min, float x_max,
                       float y_min, float y_max) {
    cv::Point2f center;

    center.x = (x_min + x_max) / 2;
    center.y = (y_min + y_max) / 2;

    return center;
}

cv::Point2f toGrid (cv::Point2f const& pt,
                    float x_min, float y_min,
                    float x_max, float y_max,
                    float x_step, float y_step) {
    cv::Point2f newPt;

    newPt.y = (y_max - (pt.y-y_min)) / y_step;
    newPt.x = (pt.x-x_min) / x_step;

    return newPt;
}

#undef M_PI
#define M_PI 3.141592654

int main (int argc, const char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " lidarData.xml" << std::endl;
        return 0;
    }

    //  Read lidar data from a file
    cv::Mat lidar_data;
    std::string filename(argv[1]);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["lidarData"]>> lidar_data;
    int nb_impacts = lidar_data.cols;
    int nb_frames = lidar_data.rows;

    //  extrinsic parameters of the lidar
    double lidar_pitch_angle = -1.*M_PI/180;
    double lidar_height = 0.47;

    //  parameters of the camera
    double uo = 256;
    double vo = 156;
    double alpha_u = 410;
    double alpha_v = 410;
    double camera_height = 1.28;
    double camera_ty = 1.8;

    //  define the parameters of the grid
    float x_min = -10.;
    float x_max = 10.;
    float y_min = 0;
    float y_max = 30.;
    float x_step = 0.2;
    float y_step =  0.2;
    int nb_cells_x = (x_max-x_min)/x_step;
    int nb_cells_y = (y_max-y_min)/y_step;

    char key = 'a';
    int frame_nb = 0;

    // Kalman filter
    float dt = 0.5;
    cv::KalmanFilter kalman(4, 2, 0);
    kalman.transitionMatrix = (cv::Mat_<float>(4, 4) << 1 , 0 , dt , 0
                                                      , 0 , 1 , 0 , dt
                                                      , 0 , 0 , 1 , 0
                                                      , 0 , 0 , 0 , 1);
    kalman.measurementMatrix = (cv::Mat_<float>(2, 4) << 1 , 0, 0, 0
                                                      , 0 , 1, 0, 0);
    cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(1));
    cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(1));
    cv::setIdentity(kalman.errorCovPost, cv::Scalar::all(0.1));

    // Initial ROI
    float x_roi_min = 4,
          x_roi_max = 7.5,
          y_roi_min = 9,
          y_roi_max = 11;

    // Kalman
    cv::Point2f predicted = cv::Vec2f(0, 0);
    cv::Vec2f speed = cv::Vec2f(0, 0);
    cv::Mat_<float> observed(2, 1);
    observed.setTo(cv::Scalar(0));
    int observed_impacts = 0;

	std::ofstream out_velocity("velocity.dat");

    while (key != 'q' && frame_nb != nb_frames) {
        //  Allocation/initialization of the grid
        cv:: Mat grid = cv::Mat::zeros(cv::Size(nb_cells_x, nb_cells_y), CV_32F);

        //  Read the stereo image
        std::ostringstream filename;
        filename<< img_directory << "left_img_"<<frame_nb<<".png";
        cv::Mat left_img = cv::imread(filename.str(), 0);
        cv::Mat left_display_img;
        cv::cvtColor(left_img, left_display_img, CV_GRAY2RGB);

        observed_impacts = 0;
        observed.setTo(cv::Scalar(0));

        // Prediction
        if (frame_nb != 0) {
            cv::Mat prediction = kalman.predict();
            predicted = cv::Point2f(prediction.at<float>(0),
                                    prediction.at<float>(1));
            speed = cv::Vec2f(prediction.at<float>(2), prediction.at<float>(3));
			out_velocity << frame_nb << ' ' << cv::norm(speed) << '\n';

            cv::Point2f center = centerBox(x_roi_min, x_roi_max,
                                           y_roi_min, y_roi_max);
            cv::Vec2f newCenter = predicted - center;
            x_roi_min += newCenter[0];
            y_roi_min += newCenter[1];
            x_roi_max += newCenter[0];
            y_roi_max += newCenter[1];
        }

        //  Process all the lidar impacts and compute the observed position
        for (int i=0; i<nb_impacts/2; ++i) {
            double x=lidar_data.at<double>(frame_nb, 2*i);
            double y=lidar_data.at<double>(frame_nb, 2*i+1);

            //  compute the grid
            if (x>x_min && x<x_max && y>y_min && y<y_max && y>0)
                grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 1.0;


            // Observation
            if (x > x_roi_min && x < x_roi_max && y > y_roi_min && y < y_roi_max) {
                observed(0) += x;
                observed(1) += y;
                observed_impacts++;
            }

            //  display on stereo image
            if (y>0) {
                double z=camera_height -(lidar_height + sqrt(x*x+y*y)*sin(lidar_pitch_angle));
                int u=(int)uo+alpha_u*(x/(y+camera_ty));
                int v=(int)vo+alpha_v*(z/(y+camera_ty));
                if (u>0 && u<left_img.cols && v>0 && v<left_img.rows) {
                    left_display_img.at<unsigned char>(v, 3*u) = 0;
                    left_display_img.at<unsigned char>(v, 3*u+1) = 0;
                    left_display_img.at<unsigned char>(v, 3*u+2) = 255;
                }
            }
        }

        // Initial prediction
		observed /= observed_impacts;
        if (frame_nb == 0) {
            kalman.statePre.at<float>(0) = observed(0);
            kalman.statePre.at<float>(1) = observed(1);
            kalman.statePre.at<float>(2) = 0;
            kalman.statePre.at<float>(3) = 0;
        }

        // Correction
        cv::Mat corrected = kalman.correct(observed);
        //std::cout << "pre: " << predicted << std::endl;
        //std::cout << "obs: " << observed << std::endl;

        // prepare the display of the grid
        cv::Mat display_grid; //  to have a RGB grid for display
        grid.convertTo(display_grid, CV_8U, 255);
        cv::cvtColor(display_grid, display_grid, CV_GRAY2RGB);

        cv::Mat display_grid_large;// to have a large grid for display
        cv::resize(display_grid, display_grid_large, cv::Size(600,600));

        // show prediction / observation
		cv::Point2f scale(600.f / nb_cells_x, 600.f / nb_cells_y);
		cv::Point2f obsCross = toGrid(cv::Point2f(observed(0), observed(1)),
			x_min, y_min, x_max, y_max,
			x_step, y_step);
		obsCross.x *= scale.x;
		obsCross.y *= scale.y;
		cv::Point2f predCross = toGrid(predicted, x_min, y_min, x_max, y_max, x_step, y_step);
		predCross.x *= scale.x;
		predCross.y *= scale.y;
		drawCross(obsCross, cv::Scalar(255, 0, 0), 5);
		drawCross(predCross, cv::Scalar(0, 255, 0), 5);

        //  show images
        cv::imshow("top view",  display_grid_large);
        cv::imshow("left image", left_display_img);

        //  Wait for the user to press a key
        frame_nb++;
        key = cv::waitKey();
    }

    return 0;
}

