/*
 *  sfs-framework
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Charles Papon - charles.papon.90@gmail.com
 *      Copyright (c) 2019 DHLAB, EPFL & HES-SO Valais-Wallis
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

Eigen::Vector3d * Viewpoint::getModelPoint(int pointIndex){
    return &model[pointIndex];
}

Eigen::Vector3d * Viewpoint::getCentroid(){
    return &centroid;
}

Eigen::Matrix3d * Viewpoint::getOrientation(){
    return &orientation;
}

Eigen::Vector3d * Viewpoint::getPosition(){
    return &position;
}

Eigen::Vector3d * Viewpoint::getDirection(unsigned int dirID){
    return &direction[dirID];
}

double Viewpoint::getDisparity(int featID){
    return disparity[featID];
}

void Viewpoint::resetFrame(){
    orientation=Eigen::Matrix3d::Identity();
    position=Eigen::Vector3d::Zero();
}

void Viewpoint::setPose(Eigen::Matrix3d newOrientation, Eigen::Vector3d newPosition){
    orientation=newOrientation;
    position=newPosition;
}

void Viewpoint::setRadius(int featID, double newRadius, double newDisparity){
    radius[featID]=newRadius;
    disparity[featID]=newDisparity;
}

void Viewpoint::computeModel() {
    for(unsigned int i(0); i < cvFeatures.size(); i++){
        model[i]=position+orientation*(direction[i]*radius[i]);
    }
}

void Viewpoint::computeCentroid(){
    centroid=Eigen::Vector3d::Zero();
    for(unsigned int i(0); i<cvFeatures.size(); i++){
        centroid+=model[i];
    }
    centroid/=cvFeatures.size();
}

void Viewpoint::allocateFeaturesFromCvFeatures(){
	features.resize(cvFeatures.size());
	for(uint32_t i = 0;i < cvFeatures.size();i++){
		features[i].disparity = 0;
		features[i].radius = 1;
		features[i].position[0] = cvFeatures[i].pt.x;
		features[i].position[1] = cvFeatures[i].pt.y;
		features[i].viewpoint = this;
		features[i].structure = NULL;
		features[i].direction = Eigen::Vector3d(0,0,0);
	}
}
