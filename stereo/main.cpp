#include <opencv2/opencv.hpp>
#include "tp_util.hpp"
#include <iostream>
#include <algorithm>
#include <cstdint>

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
	cv::Mat v_disparity_map(disp.rows, disp_max, CV_16S, cv::Scalar::all(0));

    for (int v = 0; v < disp.rows; ++v) {
		short *output = v_disparity_map.ptr<short>(v);
        for (int u = 0; u < disp.cols; ++u) {
			short d = disp.at<short>(v, u);
            if (d > 0 && (d/16.f) < disp_max) {
                output[(int)(d/16.f)]++;
            }
        }
    }

    return v_disparity_map;
}

struct Target
{
	cv::Vec3f position;
	cv::Vec4f bb;
};

void clustering(cv::Mat const &filtered_disp)
{
	cv::Mat tmp;
	cv::Mat elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
	cv::erode(filtered_disp, tmp, elem);
	cv::dilate(tmp, tmp, elem);

	cv::Mat display;
	tmp.convertTo(display, CV_8U);
	cv::imshow("Erode + dilate", display);

	cv::Mat map;
	unsigned int nbconn = segmentDisparity(tmp, map);
	std::cout << "Number of connected components: " << nbconn << '\n';
	std::vector<std::pair<cv::Vec4i, cv::Vec2f> > bbs;
	bbs.resize(nbconn);

	// TODO verify l < r and t < b
	for (int i = 0; i < bbs.size(); ++i) {
		bbs[i].first[0] = map.cols;
		bbs[i].first[2] = map.rows;
	}

	for (int i = 0; i < map.rows; ++i) {
		for (int j = 0; j < map.cols; ++j) {
			cv::Vec4i &bb = bbs[map.at<int>(i, j)].first;
			if (j < bb[0]) {
				bb[0] = j;
			}
			if (j > bb[1]) {
				bb[1] = j;
			}
			if (i < bb[2]) {
				bb[2] = i;
			}
			if (i > bb[3]) {
				bb[3] = i;
			}
			bbs[map.at<int>(i, j)].second[0] += tmp.at<short>(i, j) / 16.f;
			bbs[map.at<int>(i, j)].second[1] += 1.0f;
		}
	}

	for (unsigned int i = 0; i < nbconn; ++i) {
		bbs[i].second[0] /= bbs[i].second[1];
	}

	std::vector<Target> targets(nbconn);
	for (unsigned int i = 0; i < nbconn; ++i) {
		targets[i].position = uvTo3D(
			(bbs[i].first[0] + bbs[i].first[1]) / 2.f,
			(bbs[i].first[2] + bbs[i].first[3]) / 2.f,
			bbs[i].second[0]);
		targets[i].bb = bbs[i].first;
	}

	// filter small regions
	std::vector<Target> filtered_targets;
	for (unsigned int i = 0; i < nbconn; ++i) {
		if ((bbs[i].first[1] - bbs[i].first[0]) * (bbs[i].first[3] - bbs[i].first[2]) > 50) {
			filtered_targets.push_back(targets[i]);
		}
	}


	std::cout << "Number of filtered targets: " << filtered_targets.size() << '\n';
	// 
	for (unsigned int i = 0; i < filtered_targets.size(); ++i) {
		cv::rectangle(display,
			cv::Point(filtered_targets[i].bb[0], filtered_targets[i].bb[2]),
			cv::Point(filtered_targets[i].bb[1], filtered_targets[i].bb[3]),
			cv::Scalar(255, 255, 255));
	}
	cv::imshow("Detected targets", display);
	
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
		const short *input = stereo_img.ptr<short>(v);
		short *output = stereo_threshold_img.ptr<short>(v);
        for(int u = 0; u < stereo_img.cols; ++u) {
            float d = float(input[u]) / 16;
            cv::Vec3f coords3D = uvTo3D(u, v, d);
            // TODO: check
            if (coords3D[2] < 0.2 || coords3D[2] > 2.5) {
                output[u] = 0;
            }
        }
    }
    cv::Mat display_stereo_threshold_img;
    stereo_threshold_img.convertTo(display_stereo_threshold_img, CV_8U);

    // 3.Road/obstacles segmentation in Disparity space
    // TODO: next line segfault
	cv::Mat v_disparity_map = v_disparity(stereo_img, MAX_DISPARITY);
	cv::Mat display_v_disparity_map;
	v_disparity_map.convertTo(display_v_disparity_map, CV_8U);
	cv::imshow("V-disparity", display_v_disparity_map);
	
    // TODO: compute theta and Z0

    // Threshold
    cv::Mat v_disparity_map_threshold;
    cv::threshold(v_disparity_map, v_disparity_map_threshold, 60, 0, cv::THRESH_TOZERO); 

    // Ransac
    std::vector<cv::Point2f> points;
    for (int i = 0; i < v_disparity_map_threshold.rows; ++i) { 
         for (int j = 0; j < v_disparity_map_threshold.cols; ++j) {
			 if (v_disparity_map_threshold.at<short>(i, j) != 0)
				points.push_back(cv::Point2f(j, i)); 
         } 
    } 
    cv::Vec4f line;
    fitLineRansac(points, line);
	std::cout << line << '\n';
	float theta = atan2(line[1], line[0]);
	std::cout << theta << '\n';

    // TODO: filter disparity image
	cv::Mat filtered_disparity = stereo_img.clone();
	cv::Mat display_filtered_disparity;
	for (int v = 0; v < stereo_img.rows; ++v) {
		for (int u = 0; u < stereo_img.cols; ++u) {
			float d = stereo_img.at<short>(v, u)/16.f;
			float dd = cv::Point2f(line[0], line[1]).cross(cv::Point2f(d, v) - cv::Point2f(line[2], line[3]));
			if (dd < (0.0f)) {
				filtered_disparity.at<short>(v, u) = 0;
			}
		}
	}
	filtered_disparity.convertTo(display_filtered_disparity, CV_8U);
	cv::imshow("Filtered disparity map", display_filtered_disparity);


    // 4. Clustering
	clustering(filtered_disparity);

    // Display images and wait for a key press
    cv::imshow("left image", left_img);
    cv::imshow("right image", right_img);
    cv::imshow("stereo image", display_stereo_img);
    cv::imshow("stereo image segmented", display_stereo_threshold_img);

    cv::waitKey();

    return 0;
}

