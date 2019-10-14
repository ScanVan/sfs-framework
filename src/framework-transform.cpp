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

void Transform::pushCorrelation(Eigen::Vector3d * firstComponent, Eigen::Vector3d * secondComponent){
    correlation+=Eigen::Matrix3d(((*firstComponent)-centerFirst)*((*secondComponent)-centerSecond).transpose());
}

void Transform::pushCentroid(Eigen::Vector3d * pushFirst, Eigen::Vector3d * pushSecond){
    centerFirst +=*pushFirst;
    centerSecond+=*pushSecond;
    centerCount++;
}

void Transform::resetCorrelation(){
    correlation=Eigen::Matrix3d::Zero();
}

void Transform::resetCentroid(){
    centerFirst =Eigen::Vector3d::Zero();
    centerSecond=Eigen::Vector3d::Zero();
    centerCount=0;
}

void Transform::computeCentroid(){
    centerFirst /=double(centerCount);
    centerSecond/=double(centerCount);
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
    translation=centerSecond-rotation*centerFirst;
}

void Transform::computeFrame(Viewpoint * first, Viewpoint * second){
    Eigen::Matrix3d trotation(rotation.transpose());
    Eigen::Matrix3d *prevRotation(first->getOrientation());
    Eigen::Vector3d *prevPosition(first->getPosition());
    second->setPose( (*prevRotation)*trotation,(*prevPosition)-trotation*translation);
}
