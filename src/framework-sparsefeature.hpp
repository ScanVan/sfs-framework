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
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>

// Internal includes
#include "gms_matcher.hpp"

void akazeFeatures(cv::Mat* image, cv::Mat* mask, std::vector<cv::KeyPoint>* keypoints, cv::Mat* desc, float const threshold);

extern void gmsMatcher (	std::vector<cv::KeyPoint>* k1,
					        cv::Mat* d1,
					        cv::Size s1,
					        std::vector<cv::KeyPoint>* k2,
					        cv::Mat* d2,
					        cv::Size s2,
					        std::vector<cv::DMatch> *matches);
