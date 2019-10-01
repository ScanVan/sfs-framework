/*
 * framework-utiles.hpp
 *
 *  Created on: Sep 30, 2019
 *      Author: dolu
 */

#pragma once

#include <opencv4/opencv2/core.hpp>
#include <Eigen/Core>

Eigen::Vector3d convertCartesian2Spherical (double x, double y, int width, int height);

