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
#include <Eigen/Core>
#include <opencv4/opencv2/core.hpp>

// Internal includes
#include "framework-utiles.hpp"

// External objects
class Viewpoint;
class Structure;

// Module object
class Feature{

public: /* Need to be set back to private */
	Viewpoint *viewpoint;
	Structure *structure;
	Eigen::Vector2f position;
	Eigen::Vector3d direction;
    Eigen::Vector3d model;
	double radius;
	double disparity;
	cv::Vec3b color;

public:
    Eigen::Vector3d * getModel();
    double getRadius();
    double getDisparity();
    Viewpoint * getViewpoint();
    Structure * getStructure();
    cv::Vec3b getColor();
    void setFeature(double x, double y, int imageWidth, int imageHeight);
    void setColor(cv::Vec3b newColor);
    void setRadius(double newRadius, double newDisparity);
    void setViewpointPtr(Viewpoint * newViewpoint);
    void setStructurePtr(Structure * newStructure);
    void reset();
    void computeModel();
    void computeOriented(Eigen::Matrix3d * orientation);

};
