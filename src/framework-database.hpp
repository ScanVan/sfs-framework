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

#define DB_LOOP_BOOT_COUNT ( 3 ) /* amount of viewpoint to wait to trigger the initial boot optimisation */

#define DB_LOOP_MODE_BOOT  ( 0 ) /* initial structure */
#define DB_LOOP_MODE_LAST  ( 1 ) /* optimising last viewpoint */
//#define DB_LOOP_MODE_HEAD  ( 2 ) /* optimising active head */
#define DB_LOOP_MODE_FULL  ( 2 ) /* optimising all structure */

class Database {

//private:
public:
	std::vector<std::shared_ptr<Viewpoint>> viewpoints;
    std::vector<std::shared_ptr<Transform>> transforms;
    std::vector<std::shared_ptr<Structure>> structures;

    unsigned int sortStructTypeA;
    unsigned int sortStructTypeB;

    double configError;
    double configDisparity;
    double configRadius;
    double meanValue;
    double stdValue;
    double maxValue;

public:
    Database(double initialError, double initialDisparity, double initialRadius);
    bool getBootstrap();
    double getConfigError();
    double getError();
    double getTranslationMeanValue();
    void getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints);
	void addViewpoint(std::shared_ptr<Viewpoint> viewpoint);
    void aggregate(std::vector<std::shared_ptr<Viewpoint>> *localViewpoints, Viewpoint *newViewpoint, uint32_t *correlations);
    void prepareStructure();
    void computeModels();
    void computeCentroids(int loopState);
    void computeCorrelations(int loopState);
    void computePoses(int loopState);
    void computeFrames();
    void computeOptimals(long loopState);
    void computeRadii(long loopState);
    void computeStatistics(long loopState, double(Feature::*getValue)());
    void computeFiltersRadialClamp(int loopState);
    void computeFiltersRadialStatistics(int loopState);
    void computeFiltersDisparityStatistics(int loopState);
    void exportModel(std::string path, unsigned int major);
    void exportOdometry(std::string path, unsigned int major);
    Structure *newStructure(Viewpoint *originalViewpoint){ auto s = std::make_shared<Structure>(originalViewpoint); structures.push_back(s); return s.get();} /* need deletion */

public:
    void _displayViewpointStructures(Viewpoint *viewpoint, unsigned int structSizeMin);
    cv::Mat viewpointStructuralImage(Viewpoint *viewpoint, unsigned int structSizeMin);
    void _sanityCheck(bool inliner);
    void _sanityCheckStructure();
    void _exportState(std::string path,int major, int iter); /* need deletion */
    void _exportMatchDistribution(std::string path, unsigned int major, std::string type); /* need deletion */

};

