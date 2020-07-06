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
#include <vector>
#include <Eigen/Dense>

// Internal includes
#include "framework-feature.hpp"
#include "framework-transform.hpp"
#include "framework-viewpoint.hpp"
#include "framework-utiles.hpp"

// Define structure activity
#define STRUCTURE_REMOVE ( 0 ) /* Removed by filtering process - no more usable */
#define STRUCTURE_NORMAL ( 1 ) /* Normal structure, containing two features or more */
#define STRUCTURE_PIONER ( 2 ) /* Structure that contains all the last viewpoints in the pipeline active head (configGroup) */

// Module object
class Structure {

public: /* Need to be set back to private */
    Eigen::Vector3d position;
    std::vector<Feature*> features;
    unsigned int state;
    unsigned int start;

public:
    Structure() : position(Eigen::Vector3d::Zero()), state(STRUCTURE_REMOVE) {}
    unsigned int getFeatureCount();
    unsigned int getFeatureViewpointIndex(unsigned int featureIndex);
    bool getHasScale(unsigned int scaleGroup);
    Eigen::Vector3d * getPosition();
    unsigned int getState();
    cv::Vec3b getColor();
    void setReset();
    void addFeature(Feature * feature);
    void sortFeatures();
    void computeState(unsigned int scaleGroup, unsigned int highViewpoint);
    void computeModel();
    void computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms, unsigned int lowViewpoint);
    void computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms, unsigned int lowViewpoint);
    void computeOriented(unsigned int lowViewpoint);
    void computeOptimalPosition(unsigned int lowViewpoint);
    void computeRadius(unsigned int lowViewpoint);
    unsigned int computeDisparityMean(double * const meanValue,unsigned int lowViewpoint);
    void computeDisparityStd(double * const stdValue, double const meanValue,unsigned int lowViewpoint);
    void filterRadialRange(double lowClamp, double highClamp,unsigned int lowViewpoint);
    void filterDisparity(double limitValue,unsigned int headStart);
    void filterResize(unsigned int resize);

};
