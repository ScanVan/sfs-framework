#pragma once

#include <opencv4/opencv2/core.hpp>
#include <vector>

#include "framework-viewpoint.hpp"
#include "framework-transform.hpp"
#include "framework-match.hpp"
#include "framework-structure.hpp"
#include "framework-io.hpp"
#include "framework-parameter.hpp"

class Database {

private:
	std::vector<std::shared_ptr<Viewpoint>> viewpoints;
    std::vector<std::shared_ptr<Transform>> transforms;
    std::vector<std::shared_ptr<Match>> matches;
    std::vector<std::shared_ptr<Structure>> structures;
    Io ios;
    Parameter parameters;

public:
	void addViewpoint(std::shared_ptr<Viewpoint> viewpoint){
		viewpoints.push_back(viewpoint);
	}
};

