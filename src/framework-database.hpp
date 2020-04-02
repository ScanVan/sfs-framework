/*
 *  sfs-framework
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Charles Papon - charles.papon.90@gmail.com
 *      Copyright (c) 2019-2020 DHLAB, EPFL & HES-SO Valais-Wallis
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

// External includes
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv4/opencv2/core.hpp>
#include <omp.h>
#include <experimental/filesystem>
#include <sstream>
#include <iomanip>

// Internal includes
#include "framework-viewpoint.hpp"
#include "framework-transform.hpp"
#include "framework-structure.hpp"

// Namespaces
namespace fs = std::experimental::filesystem;

// Optimisation algorithm maximum iteration per cycle
#define DB_LOOP_MAXITER    ( 5000 )

// Minimal number of viewpoints required to start optimisation algorithm
#define DB_LOOP_BOOT_COUNT ( 3 )

// Optimisation algorithm states
#define DB_MODE_NULL       ( -1 ) /* Null mode */
#define DB_MODE_BOOT       (  0 ) /* Initial structure */
#define DB_MODE_LAST       (  1 ) /* Optimising last viewpoint */
#define DB_MODE_HEAD       (  2 ) /* Optimising active head */
#define DB_MODE_FULL       (  4 ) /* Optimising all structures */
#define DB_MODE_MASS       (  5 ) /* Only compute position of structures and strict filter */

// Module object
class Database {

public: /* Need to be set back to private */
	std::vector<std::shared_ptr<Viewpoint>> viewpoints;
    std::vector<std::shared_ptr<Transform>> transforms;
    std::vector<std::shared_ptr<Structure>> structures;

    unsigned int sortStructTypeA;
    unsigned int sortStructTypeB;
    double transformMean;
    double configError;
    double configDisparity;
    double configRadius;
    unsigned int configMatchRange;
    double meanValue;
    double stdValue;
    double maxValue;

public:
    Database(double initialError, double initialDisparity, double initialRadius, unsigned int initialMatchRange);
    bool getBootstrap();
    double getPError();
    double getDError();
    bool getCheckError( double const currentError, double const lastError );
    void getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints);
	void addViewpoint(std::shared_ptr<Viewpoint> viewpoint);
    void aggregate(std::vector<std::shared_ptr<Viewpoint>> *localViewpoints, Viewpoint *newViewpoint, uint32_t *correlations);
    void prepareFeature();
    void prepareStructure();
    void computeModels(int loopState);
    void computeCentroids(int loopState);
    void computeCorrelations(int loopState);
    void computePoses(int loopState);
    void computeNormalisePoses(int loopState);
    void computeFrames(int loopState);
    void computeOriented(long loopState);
    void computeOptimals(long loopState);
    void computeRadii(long loopState);
    void computeDisparityStatistics(long loopState);
    void filterRadialPositivity(int loopState);
    void filterRadialLimitation(int loopState);
    void filterDisparity(int loopState);
    void exportStructure(std::string path, std::string mode, unsigned int major);
    void exportPosition(std::string path, std::string mode, unsigned int major);
    void exportTransformation(std::string path, std::string mode, unsigned int major);
    Structure *newStructure(Viewpoint *originalViewpoint){ auto s = std::make_shared<Structure>(originalViewpoint); structures.push_back(s); return s.get();} /* need deletion */

public:
    void _displayViewpointStructures(Viewpoint *viewpoint, unsigned int structSizeMin);
    cv::Mat viewpointStructuralImage(Viewpoint *viewpoint, unsigned int structSizeMin);
    void _sanityCheck(bool inliner);
    void _sanityCheckStructure();
    void _sanityCheckFeatureOrder();
    void _exportState(std::string path,int major, int iter); /* need deletion */
    void _exportMatchDistribution(std::string path, unsigned int major, std::string type); /* need deletion */

};

