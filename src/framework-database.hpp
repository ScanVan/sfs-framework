#pragma once

#include "framework-viewpoint.hpp"

#include <opencv4/opencv2/core.hpp>
#include <vector>

class Database {
private:
	std::vector<std::shared_ptr<Viewpoint>> viewpoints;

public:
	void addViewpoint(std::shared_ptr<Viewpoint> viewpoint){
		viewpoints.push_back(viewpoint);
	}
};

