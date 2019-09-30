#pragma once

#include <vector>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/types.hpp>

double computeStillDistance(std::vector<cv::KeyPoint> *kp1,
							std::vector<cv::KeyPoint> *kp2,
							std::vector<cv::DMatch> *matches,
							cv::Size size);
