#pragma once

#include <Eigen/Core>

class Viewpoint;
class Structure;
class Feature{
public:
	Viewpoint *viewpoint;
	Structure *structure;
	Eigen::Vector2f position;
	Eigen::Vector3d direction;
	double radius;
	double disparity;
};
