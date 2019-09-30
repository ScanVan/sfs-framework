#pragma once


#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>


extern void akazeFeatures(	cv::Mat* image,
							cv::Mat* mask,
							std::vector<cv::KeyPoint>* keypoints,
							cv::Mat* desc);


extern void gmsMatcher (	std::vector<cv::KeyPoint>* k1,
					cv::Mat* d1,
					cv::Size s1,
					std::vector<cv::KeyPoint>* k2,
					cv::Mat* d2,
					cv::Size s2,
					std::vector<cv::DMatch> *matches);
