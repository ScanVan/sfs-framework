/*
 * framework-utiles.cpp
 *
 *  Created on: Sep 30, 2019
 *      Author: dolu
 */



#include "framework-utiles.hpp"

Eigen::Vector3d  convertCartesian2Spherical (double x, double y, int width, int height) {
// Inputs:
// x 				: cartesian x-coordinate
// y				: cartesian y-coordinate
// width			: width of the equirectangular image
// height			: height of the equirectangular image
// Outputs:
// Points<double>	: converted spherical coordinates in type Points<double>

	// Correction from Nils
	double tm1 { (x / width) * 2 * M_PI };
	double tm2 { ((y / (height-1)) - 0.5) * M_PI };

	// coordinate conversion
	double p1 { cos(tm2) * cos(tm1) };
	double p2 { cos(tm2) * sin(tm1) };
	double p3 { sin(tm2) };

	Eigen::Vector3d  p { p1, p2, p3 };



	return p;

}
