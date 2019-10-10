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
#include "framework-viewpoint.hpp"

class Transform {
//private:
public:
    Eigen::Matrix3d correlation;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    Eigen::Vector3d centroid_a;
    Eigen::Vector3d centroid_b;
    unsigned int pushCount;
public:
    Eigen::Matrix3d * getRotation();
    Eigen::Vector3d * getTranslation();
    void pushCorrelation(Eigen::Vector3d * first, Eigen::Vector3d * fcentroid, Eigen::Vector3d * second, Eigen::Vector3d * scentroid);
    void pushCentroid(Eigen::Vector3d * push_a, Eigen::Vector3d * push_b);
    void resetCorrelation();
    void resetCentroid();
    void computeCentroid();
    void computePose(Viewpoint * first, Viewpoint * second);
    void computeFrame(Viewpoint * first, Viewpoint * second);
};

