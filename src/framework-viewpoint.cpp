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

int Viewpoint::getIndex(){
    return index;
}

//Eigen::Vector3d * Viewpoint::getModelPoint(unsigned int ID){
//    return features[ID].getModel();
//}

Eigen::Vector3d * Viewpoint::getCentroid(){
    return &centroid;
}

Eigen::Matrix3d * Viewpoint::getOrientation(){
    return &orientation;
}

Eigen::Vector3d * Viewpoint::getPosition(){
    return &position;
}

Eigen::Vector3d * Viewpoint::getDirection(unsigned int ID){
    return features[ID].getDirection();
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

void Viewpoint::setRadius(unsigned int ID, double newRadius){
    features[ID].setRadius(newRadius);
}

void Viewpoint::computeModel() {
    for(unsigned int i(0); i<features.size(); i++){
        features[i].computeModel();
    }
}

void Viewpoint::computeCentroid(){
    centroid=Eigen::Vector3d::Zero();
    for(unsigned int i(0); i<features.size(); i++){
        centroid+=(*features[i].getModel());
    }
    centroid/=double(features.size());
}

void Viewpoint::allocateFeaturesFromCvFeatures(){
	features.resize(cvFeatures.size());
	for(uint32_t i = 0;i < cvFeatures.size();i++){
        features[i].setFeature(cvFeatures[i].pt.x, cvFeatures[i].pt.y, width, height);
        features[i].setRadius(1.);
        features[i].setViewpointPtr(this);
        features[i].setStructurePtr(NULL);
	}
}
