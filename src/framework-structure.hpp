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
#include "framework-transform.hpp"
#include "framework-viewpoint.hpp"

class Structure {
//private:
public:
    Eigen::Vector3d position;
    double disparity;
    std::vector<Feature*> features;
    //std::vector< long > vplink; // to be removed
    //std::vector< long > ftlink; // to be removed
public:
    Eigen::Vector3d * getPosition();
    unsigned int getFeatureCount();
    double getRadius(unsigned int featureID);
    double getDisparity(unsigned int featureID);
    std::vector< Feature* > *getFeatures(){ return &features; }
    void computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms);
    void computeOptimalPosition();
    void computeRadius(std::vector<std::shared_ptr<Viewpoint>> & viewpoints);
    bool computeFilter(double dispSD, double radMean, double radSD, double dispTolerence, double radTolerence);
    //bool computeFilter(double dispTolerence, double triTolerence, double disparitySD);
    //bool computeFilterTriangulation(double triTolerence);
    //bool computeFilterRadius(double radTolerence, double radiusMean, double radiusSD);
    //bool computeFilterDisparity(double dispTolerence, double disparitySD);
    void addFeature(Feature* feature) {feature->structure = this; features.push_back(feature); }
};
