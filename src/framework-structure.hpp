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

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include "framework-feature.hpp"
#include "framework-transform.hpp"
#include "framework-viewpoint.hpp"

class Structure {
public:
    Eigen::Vector3d position;
    std::vector<Feature*> features;
    Viewpoint *originalViewpoint;

public:
    Structure(Viewpoint *originalViewpoint) : originalViewpoint(originalViewpoint) {}
    Eigen::Vector3d * getPosition();
    int getFeaturesCount();
    bool getBootstrap(int lastViewpointIndex);
    bool getLastViewpointCreated(unsigned int lastViewpointIndex);
    bool getHasLastViewpoint(int lastViewpointIndex);
    void setFeaturesState();
    void addFeature(Feature * feature);
    void computeModel();
    void computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms);
    void computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms);
    void computeOptimalPosition(unsigned int ignoreViewpoint);
    void computeRadius();

    bool filterRadiusClamp2(double clampValue);
    bool filterRadiusClamp(double clampValue, int indexRange);

    bool filterRadiusStatistics(double meanValue, double stdValue, int indexRange);

    bool filterDisparityStatistics(double stdValue, int indexRange);
    bool filterDisparityStatistics2(double stdValue, int indexRange);

    bool filterTriangulation(double const minAngle, double const maxAngle);
    Viewpoint *getOriginalViewpoint() { return originalViewpoint; }
    std::vector< Feature* > *getFeatures(){ return &features; } /* do not create methods for development realated function / or specify it clearly - will need to desapear */

};
