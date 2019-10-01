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

#include <string>
#include <Eigen/Dense>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>

class Viewpoint {
private:
    std::string uid;
	cv::Mat image;
	std::vector<cv::KeyPoint> features;
	cv::Mat descriptor;
    std::vector< Eigen::Vector3d > direction;
    std::vector< Eigen::Vector3d > model;
    std::vector< double > radius;
    std::vector< double > disparity;
    Eigen::Matrix3d orientation;
    Eigen::Vector3d position;
    Eigen::Vector3d centroid;
public:
	void setImage(cv::Mat &image){this->image = image;}
	cv::Mat* getImage(){return &this->image;}
	void setFeatures(std::vector<cv::KeyPoint> &image){this->features = features;}
	std::vector<cv::KeyPoint>* getFeatures(){return &this->features;}
	void setDescriptor(cv::Mat &image){this->descriptor = descriptor;}
	cv::Mat* getDescriptor(){return &this->descriptor;}
	void setPosition(Eigen::Vector3d position){this->position = position;}
    Eigen::Vector3d * getModelPoint(int pointIndex);
    Eigen::Vector3d * getCentroid();
    Eigen::Matrix3d * getOrientation();
    Eigen::Vector3d * getPosition();
    Eigen::Vector3d * getDirection(unsigned int dirID);
    double getDisparity(int featID);
    void resetFrame();
    void setPose(Eigen::Matrix3d newOrientation, Eigen::Vector3d newPosition);
    void setRadius(int featID, double radius, double disparity);
    void computeModel();
    void computeCentroid();
    //void computeFrame(Viewpoint * previous, Transform * transform);
//	GETSET(cv::Mat, Image);
//	GETSET(std::vector<cv::KeyPoint> , Features);
//	GETSET(cv::Mat, Descriptor);
};
