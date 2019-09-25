#pragma once


#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>


extern void akazeFeatures(	cv::Mat* image,
							cv::Mat* mask,
							std::vector<cv::KeyPoint>* keypoints,
							cv::Mat* desc);
