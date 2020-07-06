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

#include "framework-viewpoint.hpp"

unsigned int Viewpoint::getIndex(){

    // Return viewpoint index
    return index;

}

Eigen::Matrix3d * Viewpoint::getOrientation(){

    // Return orientation matrix pointer
    return &orientation;

}

Eigen::Vector3d * Viewpoint::getPosition(){

    // Return position vector pointer
    return &position;

}

void Viewpoint::resetFrame(){

    // Reset orientation and position
    orientation=Eigen::Matrix3d::Identity();
    position=Eigen::Vector3d::Zero();

}

void Viewpoint::setIndex(unsigned int newIndex){

    // Assign viewpoint index
    index=newIndex;

}

bool Viewpoint::setImage(std::string imagePath, double imageScale){

    // Import viewpoint image
    image = cv::imread(imagePath, cv::IMREAD_COLOR);

    // Check status
    if(image.empty()==false){

        // Apply image scale factor
        cv::resize(image, image, cv::Size(), imageScale, imageScale, cv::INTER_AREA);

        // Assign image size
        width=image.cols;
        height=image.rows;

        return true;

    } else {
        return false;
    }
}

void Viewpoint::setPose(Eigen::Matrix3d newOrientation, Eigen::Vector3d newPosition){

    // Assign orientation and position
    orientation=newOrientation;
    position=newPosition;

}

void Viewpoint::allocateFeaturesFromCvFeatures(){
    
    // Allocate features
	for(uint32_t i = 0;i < cvFeatures.size();i++){
    
        // Instance feature
	    auto feature = new Feature();

        // Initialise feature
        feature->setFeature(cvFeatures[i].pt.x, cvFeatures[i].pt.y, width, height);
        feature->setRadius(1., 0.);
        feature->setViewpointPtr(this);
        feature->setStructurePtr(NULL);
        feature->setColor(image.at<cv::Vec3b>(cvFeatures[i].pt.y, cvFeatures[i].pt.x));

        // Push feature
        features.push_back(feature);

	}

}

