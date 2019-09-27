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

# ifndef __FRAMEWORK_VIEWPOINT__
# define __FRAMEWORK_VIEWPOINT__

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
    std::vector< double > radius;
    std::vector< double > disparity;
    Eigen::Vector3d position;
public:
	void setImage(cv::Mat &image){this->image = image;}
	cv::Mat* getImage(){return &this->image;}
	void setFeatures(std::vector<cv::KeyPoint> &image){this->features = features;}
	std::vector<cv::KeyPoint>* getFeatures(){return &this->features;}
	void setDescriptor(cv::Mat &image){this->descriptor = descriptor;}
	cv::Mat* getDescriptor(){return &this->descriptor;}
//	GETSET(cv::Mat, Image);
//	GETSET(std::vector<cv::KeyPoint> , Features);
//	GETSET(cv::Mat, Descriptor);
};

# endif
