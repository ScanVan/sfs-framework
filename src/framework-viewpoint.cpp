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
	features.resize(cvFeatures.size());
	for(uint32_t i = 0;i < cvFeatures.size();i++){
        features[i].setFeature(cvFeatures[i].pt.x, cvFeatures[i].pt.y, width, height);
        features[i].setRadius(1., 0.);
        features[i].setViewpointPtr(this);
        features[i].setStructurePtr(NULL);
	}
}

double Viewpoint::getSecondFrom(Viewpoint *ref){
	double s = this->time - ref->time;
	s += 1e-6*(this->microsecond - ref->microsecond);
	return s;
}

////

double Viewpoint::getDisparityMean(){
    return dispMean;
}

double Viewpoint::getDisparitySD(){
    return dispSD;
}

double Viewpoint::getdistReference(){
    return distReference;
}

void Viewpoint::resetStatistics(){
    dispMean=0.;
    dispSD=0.;
    dispCount=0;
}

void Viewpoint::pushStatisticsMean(double pushValue){
    dispMean+=pushValue;
    dispCount++;
}

void Viewpoint::computeStatisticsMean(){
    dispMean/=double(dispCount);
    std::cerr << "MN(old) : (" << index << ") : " << dispMean << std::endl;
}

void Viewpoint::pushStatisticsSD(double pushValue){
    pushValue-=dispMean;
    dispSD+=pushValue*pushValue;
}

void Viewpoint::computeStatisticsSD(){
    dispSD=std::sqrt(dispSD/double(dispCount-1));
    std::cerr << "SD(old) : (" << index << ") : " << dispSD << std::endl;
}

void Viewpoint::setReferenceDistance(double newReference){
    distReference=newReference;
}

void Viewpoint::computeStatistics(){
    unsigned long count(0);
    double component(0.);
    dispMean=0.;
    for(auto & element: features){
        if(element.getState()==true){
            dispMean+=element.getDisparity();
            count++;
        }
    }
    dispMean/=double(count);

    dispSD=0.;
    for(auto & element: features){
        if(element.getState()==true){
            component=element.getDisparity()-dispMean;
            dispSD+=component*component;
        }
    }
    dispSD=std::sqrt(dispSD/double(count-1));
    std::cerr << "MN(new) : (" << index << ") : " << dispMean << std::endl;
    std::cerr << "SD(new) : (" << index << ") : " << dispSD << std::endl;
}

