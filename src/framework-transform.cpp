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

Eigen::Matrix3d * Transform::getRotation(){
    return &rotation;
}

Eigen::Vector3d * Transform::getTranslation(){
    return &translation;
}

void Transform::pushCorrelation(Eigen::Vector3d * first, Eigen::Vector3d * fcentroid, Eigen::Vector3d * second, Eigen::Vector3d * scentroid){
    correlation+=Eigen::Matrix3d(((*first)-centroid_a)*((*second)-centroid_b).transpose());
}

void Transform::pushCentroid(Eigen::Vector3d * push_a, Eigen::Vector3d * push_b){
    centroid_a+=*push_a;
    centroid_b+=*push_b;
    pushCount++;
}

void Transform::resetCorrelation(){
    correlation=Eigen::Matrix3d::Zero();
}

void Transform::resetCentroid(){
    centroid_a=Eigen::Vector3d::Zero();
    centroid_b=Eigen::Vector3d::Zero();
    pushCount=0;
}

void Transform::computeCentroid(){
    centroid_a/=double(pushCount);
    centroid_b/=double(pushCount);
}

void Transform::computePose(Viewpoint * first, Viewpoint * second){
    Eigen::JacobiSVD<Eigen::Matrix3d> svd( correlation, Eigen::ComputeFullU | Eigen::ComputeFullV );
    rotation=svd.matrixV()*svd.matrixU().transpose();
    if (rotation.determinant()<0){
        std::cerr << "SVD fault" << std::endl;
        Eigen::Matrix3d correction(Eigen::Matrix3d::Identity());
        correction(3,3)=-1;
        rotation=(svd.matrixV()*correction)*svd.matrixU().transpose();
    }
    translation=centroid_b-rotation*centroid_a;
}

void Transform::computeFrame(Viewpoint * first, Viewpoint * second){
    Eigen::Matrix3d trotation(rotation.transpose());
    Eigen::Matrix3d *prevRotation(first->getOrientation());
    Eigen::Vector3d *prevPosition(first->getPosition());
    second->setPose( (*prevRotation)*trotation,(*prevPosition)-trotation*translation);
}
