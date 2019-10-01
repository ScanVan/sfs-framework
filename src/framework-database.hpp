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

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv4/opencv2/core.hpp>
#include "framework-viewpoint.hpp"
#include "framework-transform.hpp"
#include "framework-match.hpp"
#include "framework-io.hpp"
#include "framework-structure.hpp"
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
    Io * getIos() {
        return & this->ios;
    }
    Parameter * getParameter() {
        return & this->parameters;
    }
    void setPath(std::string recordPath, std::string modelPath);
    void computeModels();
    void computeCorrelations();
    void computePoses();
    void computeFrame();
    void computeRadius();
    double computeError();
    void exportModel();
    void exportOdometry();
};

