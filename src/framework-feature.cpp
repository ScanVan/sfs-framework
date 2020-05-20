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

#include "framework-feature.hpp"

Eigen::Vector3d * Feature::getDirection(){

    // Return feature direction vector pointer
    return &direction;

}

Eigen::Vector3d * Feature::getModel(){

    // Return feature model vector pointer
    return &model;

}

double Feature::getRadius(){

    // Return feature radius
    return radius;

}

double Feature::getDisparity(){

    // Return feature disparity
    return disparity;

}

Viewpoint * Feature::getViewpoint(){
    
    // Return feature assigned viewpoint pointer
    return viewpoint;

}

Structure * Feature::getStructure(){

    // Return feature assigned structure pointer
    return structure;

}

cv::Vec3b Feature::getColor(){

    // Return feature color
    return color;

}

void Feature::setFeature(double x, double y, int imageWidth, int imageHeight){

    // Assign position on image
    position=Eigen::Vector2f(x,y);

    // Compute direction vector
    direction=utiles_direction(x,y,imageWidth,imageHeight);

}

void Feature::setColor(cv::Vec3b newColor){
    
    // Assign feature color
    color=newColor;

}

void Feature::setRadius(double newRadius, double newDisparity){

    // Assign radius
    radius=newRadius;

    // Compute and assign normalised disparity
    disparity=newDisparity/newRadius;

}

void Feature::setViewpointPtr(Viewpoint * newViewpoint){

    // Update feature assigned viewpoint
    viewpoint=newViewpoint;

}

void Feature::setStructurePtr(Structure * newStructure){

    // Update feature assigned viewpoint
    structure=newStructure;

}

void Feature::reset(){

    // Reset feature
    setRadius(1., 0.);

}

void Feature::computeModel(){

    // Compute feature model
    model=direction*radius;

}

void Feature::computeOriented(Eigen::Matrix3d * orientation){

    // Compute feature direction in absolute frame
    model=(*orientation)*direction;

}

