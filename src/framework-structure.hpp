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
#include <Eigen/Dense>
#include <vector>
#include <limits>

// Internal includes
#include "framework-feature.hpp"
#include "framework-transform.hpp"
#include "framework-viewpoint.hpp"

// Module object
class Structure {
public: /* Need to be set back to private */
    Eigen::Vector3d position;
    std::vector<Feature*> features;
    Viewpoint *originalViewpoint; /* no more needed */
    bool optimised;
    bool filtered;

public:
    Structure() : position(Eigen::Vector3d::Zero()), originalViewpoint(NULL), optimised(false), filtered(true) {}
    Structure(Viewpoint *originalViewpoint) : position(Eigen::Vector3d::Zero()), originalViewpoint(originalViewpoint), optimised(false), filtered(true) {}  /* no more needed - using standard constructor */
    Eigen::Vector3d * getPosition();
    bool getOptimised();
    bool getFiltered();
    int getFeaturesCount();
    bool getHasLastViewpoint(int lastViewpointIndex);
    bool getHasScale(unsigned int configGroup);
    void setReset();
    void setFeaturesState();
    void addFeature(Feature * feature);
    void sortFeatures();
    void computeModel();
    void computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms);
    void computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms);
    void computeOriented();
    void computeOptimalPosition();
    void computeRadius();
    unsigned int computeDisparityMean(double * const meanValue);
    void computeDisparityStd(double * const stdValue, double const meanValue);
    void computeDisparityMax(double * const maxValue);
    void filterRadialRange(double lowClamp, double highClamp);
    void filterDisparity(double limitValue);

    void filterExperimental(double minValue);

    Viewpoint *getOriginalViewpoint() { return originalViewpoint; }
    std::vector< Feature* > *getFeatures(){ return &features; } /* do not create methods for development realated function / or specify it clearly - will need to desapear */

};
