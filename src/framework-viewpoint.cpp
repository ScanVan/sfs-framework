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

int Viewpoint::getIndex(){
    return index;
}

Eigen::Matrix3d * Viewpoint::getOrientation(){
    return &orientation;
}

Eigen::Vector3d * Viewpoint::getPosition(){
    return &position;
}

void Viewpoint::resetFrame(){
    orientation=Eigen::Matrix3d::Identity();
    position=Eigen::Vector3d::Zero();
}

void Viewpoint::setIndex(int newIndex){
    index=newIndex;
}

void Viewpoint::setImageDimension(int newWidth, int newHeight){
    width=newWidth;
    height=newHeight;
}

void Viewpoint::setPose(Eigen::Matrix3d newOrientation, Eigen::Vector3d newPosition){
    orientation=newOrientation;
    position=newPosition;
}

void Viewpoint::allocateFeaturesFromCvFeatures(){
	for(uint32_t i = 0;i < cvFeatures.size();i++){
	    auto feature = new Feature();
        feature->setFeature(cvFeatures[i].pt.x, cvFeatures[i].pt.y, width, height);
        feature->setRadius(1., 0.);
        feature->setViewpointPtr(this);
        feature->setStructurePtr(NULL);
        auto color = image.empty() ? cv::Vec3b(255,255,255) : image.at<cv::Vec3b>(cvFeatures[i].pt.y, cvFeatures[i].pt.x);
        feature->setColor(color);
        features.push_back(feature);
	}
}

double Viewpoint::getSecondFrom(Viewpoint *ref){
	double s = this->time - ref->time;
	s += 1e-6*(this->microsecond - ref->microsecond);
	return s;
}
