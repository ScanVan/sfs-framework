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

#include "framework-transform.hpp"

double Transform::getError(){

    // Return error between translation and pushed translation
    return (push-translation).norm();

}

Eigen::Matrix3d * Transform::getRotation(){

    // Return rotation pointer
    return &rotation;

}

Eigen::Vector3d * Transform::getTranslation(){

    // Return translation pointer
    return &translation;

}

double Transform::getScale(){

    // Return scale factor between translation and its previous computation
    return translation.norm()/scale;

}

void Transform::setTranslationScale(double scaleFactor){

    // Apply scale factor on translation
    translation/=scaleFactor;

}

void Transform::setScale(){

    // Push translation norm
    scale=translation.norm();

}

void Transform::pushCorrelation(Eigen::Vector3d * firstComponent, Eigen::Vector3d * secondComponent){

    // Compute correlation component
    # pragma omp critical
    correlation+=((*firstComponent)-centerFirst)*((*secondComponent)-centerSecond).transpose();

}

void Transform::pushCentroid(Eigen::Vector3d * pushFirst, Eigen::Vector3d * pushSecond){

    // Push centroid component
    # pragma omp critical
    {
    centerFirst +=*pushFirst;
    centerSecond+=*pushSecond;
    count++;
    }

}

void Transform::resetCorrelation(){

    // Reset correlation matrix
    correlation=Eigen::Matrix3d::Zero();

}

void Transform::resetCentroid(){

    // Reset centroids
    centerFirst =Eigen::Vector3d::Zero();
    centerSecond=Eigen::Vector3d::Zero();
    count=0;

}

void Transform::computeCentroid(){

    // Compute centroids
    centerFirst /=double(count);
    centerSecond/=double(count);

}

void Transform::computePose(){

    // Compute SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(correlation,Eigen::ComputeFullU|Eigen::ComputeFullV);

    // Compute rotation
    rotation=svd.matrixV()*svd.matrixU().transpose();

    // Stability mechanism - In some cases, the rotation matrix has an inversion that has to be removed
    if (rotation.determinant()<0){
        std::cerr << "SVD fault" << std::endl;
        Eigen::Matrix3d correctV(svd.matrixV());
        correctV(0,2)=-correctV(0,2);
        correctV(1,2)=-correctV(1,2);
        correctV(2,2)=-correctV(2,2);
        rotation=correctV*svd.matrixU().transpose();
    }

    // Push translation - Needed to track error
    push=translation;

    // Compute translation
    translation=centerSecond-rotation*centerFirst;

}

void Transform::computeFrame(Viewpoint * first, Viewpoint * second){

    // Compute oriented rotation according to first viewpoint (already in absolute frame)
    Eigen::Matrix3d oriented((*first->getOrientation())*rotation.transpose());

    // Compute position and orientation of second viewpoint in absolute frame
    second->setPose(oriented,(*first->getPosition())-oriented*translation);

}

