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

#include "framework-transform.hpp"

void Transform::pushCorrelation(Eigen::Vector3d * first, Eigen::Vector3d * fcentroid, Eigen::Vector3d * second, Eigen::Vector3d * scentroid){
    Eigen::Matrix3d local(((*first)-(*fcentroid))*((*second)-(*scentroid)).transpose());
    correlation+=local;
}

void Transform::resetCorrelation(){
    correlation=Eigen::Matrix3d::Zero();
}

void Transform::computePose(Viewpoint * first, Viewpoint * second){
    Eigen::JacobiSVD<Eigen::Matrix3d> svd( correlation, Eigen::ComputeFullU | Eigen::ComputeFullV );
    rotation=svd.matrixV()*svd.matrixU().transpose();
    if (rotation.determinant()<0){
        Eigen::Matrix3d correction(Eigen::Matrix3d::Identity());
        correction(3,3)=-1;
        rotation=(svd.matrixV()*correction)*svd.matrixU().transpose();
    }
    translation=(*second->getCentroid())-rotation*(*first->getCentroid());
}

void Transform::computeFrame(Viewpoint * first, Viewpoint * second){
    Eigen::Matrix3d trotation(rotation.transpose());
    Eigen::Matrix3d *prevRotation(first->getOrientation());
    Eigen::Vector3d *prevPosition(first->getPosition());
    second->setPose( (*prevRotation)*trotation,(*prevPosition)-trotation*translation);
}
