/*
 * framework-utiles.hpp
 *
 *  Created on: Sep 30, 2019
 *      Author: dolu
 */

#ifndef SRC_FRAMEWORK_UTILES_HPP_
#define SRC_FRAMEWORK_UTILES_HPP_

#include <opencv4/opencv2/core.hpp>
#include <Eigen/Core>

Eigen::Vector3d convertCartesian2Spherical (double x, double y, int width, int height);


#endif /* SRC_FRAMEWORK_UTILES_HPP_ */
