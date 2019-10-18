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
#include <cmath>
#include <opencv4/opencv2/core.hpp>
#include "framework-viewpoint.hpp"
#include "framework-transform.hpp"
#include "framework-structure.hpp"

class Database {

//private:
public:
	std::vector<std::shared_ptr<Viewpoint>> viewpoints;
    std::vector<std::shared_ptr<Transform>> transforms;
    std::vector<std::shared_ptr<Structure>> structures;

    unsigned long configBootstrap;
    double configError;
    unsigned long configStructure;
    double configDisparity;
    double configRadiusMin;
    double configRadiusMax;

public:
    Database(unsigned long initialBootstrap, double initialError, unsigned long initialStructure, double initialDisparity, double initialRadiusMin, double initialRadiusMax);
    int getViewpointCount();
    double getConfigError();
    double getError();
    void getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints);
	void addViewpoint(std::shared_ptr<Viewpoint> viewpoint);
    void aggregate(std::vector<std::shared_ptr<Viewpoint>> *localViewpoints, Viewpoint *newViewpoint, uint32_t *correlations);
    void computeModels();
    void computeCorrelations();
    void computeCentroids();
    void computePoses();
    void computeFrames();
    void computeOptimals();
    void computeRadii();
    void computeStatistics();
//    void deleteAndUnlinkStructure(int id); /* need decision */
    void computeFilters();
    void extrapolateViewpoint(Viewpoint * v);
    void extrapolateStructure();
    void exportModel(std::string path, unsigned int major);
    void exportOdometry(std::string path, unsigned int major);
    Structure *newStructure(){ auto s = std::make_shared<Structure>(); structures.push_back(s); return s.get();} /* need deletion */

public:
    void _displayViewpointStructures(Viewpoint *viewpoint);
    void _sanityCheck(bool inliner);
    void _exportState(std::string path,int major, int iter);
};

