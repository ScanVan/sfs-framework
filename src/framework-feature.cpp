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

#include "framework-feature.hpp"

Eigen::Vector3d * Feature::getDirection(){
    return &direction;
}

Eigen::Vector3d * Feature::getModel(){
    return &model;
}

double Feature::getRadius(){
    return radius;
}

double Feature::getDisparity(){
    return disparity;
}

Viewpoint * Feature::getViewpoint(){
    return viewpoint;
}

Structure * Feature::getStructure(){
    return structure;
}

cv::Vec3b Feature::getColor(){
    return color;
}

void Feature::setFeature(double x, double y, int imageWidth, int imageHeight){
    position=Eigen::Vector2f(x,y);
    direction=convertCartesian2Spherical(x,y,imageWidth,imageHeight);
}

void Feature::setColor(cv::Vec3b colorValue){
    color = colorValue;
}

void Feature::setRadius(double newRadius, double newDisparity){
    radius=newRadius;
    disparity=newDisparity/newRadius;
}

void Feature::setViewpointPtr(Viewpoint * newViewpoint){
    viewpoint=newViewpoint;
}

void Feature::setStructurePtr(Structure * newStructure){
    structure=newStructure;
}

void Feature::setDirection(Eigen::Vector3d newDirection){
    direction=newDirection;
}

void Feature::reset(){
    setRadius(1., 0.);
}

void Feature::computeModel(){
    model=direction*radius;
}

