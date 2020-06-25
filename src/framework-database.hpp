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
#define DB_LOOP_MAXITER    ( 512 )

// Optimisation algorithm states
#define DB_MODE_NULL       ( -1 ) /* Null mode */
#define DB_MODE_BOOT       (  0 ) /* Initial structure */
#define DB_MODE_LAST       (  1 ) /* Optimising last viewpoint */
#define DB_MODE_HEAD       (  2 ) /* Optimising active head */
#define DB_MODE_FULL       (  4 ) /* Optimising all structures */ /* Need deletion */
#define DB_MODE_MASS       (  5 ) /* Only compute position of structures and strict filter */

// Module object
class Database {

public: /* Need to be set back to private */
	std::vector<std::shared_ptr<Viewpoint>> viewpoints;
    std::vector<std::shared_ptr<Transform>> transforms;
    std::vector<std::shared_ptr<Structure>> structures;

    double configError;
    double configErrorDisparity;
    double configRadius;
    double configDenseDisparity;

    unsigned int configGroup;
    unsigned int configMatchRange;

    double transformMean;
    double meanValue;
    double stdValue;

    unsigned int rangeVlow;  /* Viewpoints range first index */
    unsigned int rangeVhigh; /* Viewpoints range last index */
    unsigned int rangeTlow;  /* Transformations range first index */
    unsigned int rangeThigh; /* Transformations range last index */
    unsigned int rangeSlow;  /* Structures range first index */
    unsigned int rangeShigh; /* Structures range last index */
    unsigned int stateStructure; /* Structure state */

public:
    Database(double initialError, double initialErrorDisparity, double initialRadius, unsigned int initialGroup, unsigned int initialMatchRange, double initialDenseDisparity);
    bool getBootstrap();
    unsigned int getGroup();
    bool getError(int loopState, int loopMajor, int loopMinor);
    void getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints);
	void addViewpoint(std::shared_ptr<Viewpoint> viewpoint);
    void aggregate(std::vector<std::shared_ptr<Viewpoint>> *localViewpoints, Viewpoint *newViewpoint, uint32_t *correlations);
    int prepareState(int pipeState);
    void prepareStructures();
    void prepareTransforms();
    void expungeStructures();
    void broadcastScale();
    void computeModels(int loopState);
    void computeCentroids(int loopState);
    void computeCorrelations(int loopState);
    void computePoses(int loopState);
    void computeNormalisePoses(int loopState);
    void computeFrames(int loopState);
    void computeOriented(int loopState);
    void computeOptimals(int loopState);
    void computeRadii(int loopState);
    void computeDisparityStatistics(int loopState);
    void filterRadialRange(int loopState);
    void filterDisparity(int loopState);
    void exportStructure(std::string path, std::string mode, unsigned int major, unsigned int group);
    void exportPosition(std::string path, std::string mode, unsigned int major);
    void exportTransformation(std::string path, std::string mode, unsigned int major);
    void exportConstraint(std::string path, std::string mode, unsigned int major, unsigned int group);
    Structure *newStructure(){ auto s = std::make_shared<Structure>(); structures.push_back(s); return s.get();}

public:

    // Devlopment features
    void _exportState(std::string path,int major, int iter);
    void _exportStructureModel(std::string path, unsigned int major);

};

