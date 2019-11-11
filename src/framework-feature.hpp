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

#pragma once

#include <Eigen/Core>
#include "framework-utiles.hpp"
#include <opencv2/core.hpp>

class Viewpoint;
class Structure;

class Feature{
public:
	Viewpoint *viewpoint;
	Structure *structure;
	Eigen::Vector2f position;
	Eigen::Vector3d direction;
    Eigen::Vector3d model;
	double radius;
	double disparity;
	uint32_t inliner; /* redundant with selection per features count and deletion trough filtering ? + encapsulation fault */
	cv::Vec3b color;

public:
    Eigen::Vector3d * getDirection();
    Eigen::Vector3d * getModel();
    double getRadius();
    double getDisparity();
    cv::Vec3b getColor();
    Viewpoint * getViewpoint();
    Structure * getStructure();
    void setFeature(double x, double y, int imageWidth, int imageHeight);
    void setRadius(double newRadius, double newDisparity);
    void setColor(cv::Vec3b color);
    void setViewpointPtr(Viewpoint * newViewpoint);
    void setStructurePtr(Structure * newStructure);
    void setDirection(Eigen::Vector3d newDirection); /* only used for synthetic models */
    void reset();
    void computeModel();
};
