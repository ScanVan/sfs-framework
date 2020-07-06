/*
 *  sfs-framework
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Charles Papon - charles.papon.90@gmail.com
 *      Copyright (c) 2019-2020 DHLAB, EPFL & HES-SO Valais-Wallis
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// External includes
#include <iostream>
#include <cmath>
#include <string>
#include <experimental/filesystem>
#include <Eigen/Core>
#include <opencv4/opencv2/core.hpp>

// Internal includes
#include "gms_matcher.hpp"

// Namespaces
namespace fs = std::experimental::filesystem;

template<typename T> std::pair<bool, int> findInVector(const std::vector<T> &vecOfElements, const T &element) {

    std::pair<bool, int> result;

    // Find given element in vector
    auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);

    if (it != vecOfElements.end()) {
        result.second = distance(vecOfElements.begin(), it);
        result.first = true;
    } else {
        result.first = false;
        result.second = -1;
    }

    return result;

}

void utilesDirectories(std::string rootPath, std::string modeName);

Eigen::Vector3d utilesDirection(double x, double y, int width, int height);

void utilesAKAZEFeatures(cv::Mat* image, cv::Mat* mask, std::vector<cv::KeyPoint>* keypoints, cv::Mat* desc, float const threshold);

void utilesGMSMatcher(std::vector<cv::KeyPoint>* k1, cv::Mat* d1, cv::Size s1, std::vector<cv::KeyPoint>* k2, cv::Mat* d2, cv::Size s2, std::vector<cv::DMatch> *matches);

double utilesDetectMotion(std::vector<cv::KeyPoint> *kp1, std::vector<cv::KeyPoint> *kp2, std::vector<cv::DMatch> *matches, cv::Size size);

double bilinear_sample(double *p, double x, double y, int width);

float bilinear_sample(float *p, float x, float y, int width);

