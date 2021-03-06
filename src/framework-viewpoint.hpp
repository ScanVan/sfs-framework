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
#include <string>
#include <Eigen/Dense>
#include <opencv4/opencv2/core.hpp>

// Internal includes
#include "framework-feature.hpp"

// Module object
class Viewpoint {

public: /* Need to be set back to private */
    std::string uid;
    unsigned int index;
	cv::Mat image;
    int width;
    int height;
	std::vector<Feature*> features;
	std::vector<cv::KeyPoint> cvFeatures;
	cv::Mat cvDescriptor;
    Eigen::Matrix3d orientation;
    Eigen::Vector3d position;

public:
    unsigned int getIndex();
    cv::Mat * getImage();
    std::vector<cv::KeyPoint> * getCvFeatures();
    Feature * getFeatureFromCvIndex(int index);
    cv::Mat * getCvDescriptor();
    Eigen::Matrix3d * getOrientation();
    Eigen::Vector3d * getPosition();
    void releaseImage();
    void resetFrame();
    void addFeature(Feature * newFeature);
    void setIndex(unsigned int newIndex);
    bool setImage(std::string imagePath, double imageScale);
    void setPose(Eigen::Matrix3d newOrientation, Eigen::Vector3d newPosition);
    void setPosition(Eigen::Vector3d newPosition);
    void allocateFeaturesFromCvFeatures();  

};
