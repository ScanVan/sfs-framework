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

# include "framework-structure.hpp"

Eigen::Vector3d * Structure::getPosition(){
    return &position;
}

double Structure::getDisparity(){
    return disparity;
}

void Structure::computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms){
    int i_index(0), j_index(0);
    for(unsigned int i(0); i<features.size(); i++){
        i_index=features[i]->getViewpoint()->getIndex();
        for(unsigned int j(0); j<features.size(); j++){
            j_index=features[j]->getViewpoint()->getIndex();
            if (i_index-j_index==1){
                transforms[j_index]->pushCorrelation(
                    features[j]->getModel(), features[j]->getViewpoint()->getCentroid(),
                    features[i]->getModel(), features[i]->getViewpoint()->getCentroid()
                );
            }
        }
    }
}

void Structure::computeOptimalPosition(){
    Eigen::Matrix3d macc(Eigen::Matrix3d::Zero());
    Eigen::Vector3d vacc(Eigen::Vector3d::Zero());
    for(unsigned int i(0); i<features.size(); i++){
        Eigen::Vector3d dirVector((*features[i]->getViewpoint()->getOrientation()).transpose()*(*features[i]->getDirection()));
        Eigen::Matrix3d weight(Eigen::Matrix3d::Identity()-dirVector*dirVector.transpose());
        macc+=weight;
        vacc+=weight*(*features[i]->getViewpoint()->getPosition());
    }
    position=macc.inverse()*vacc;
}

void Structure::computeRadius(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    for(unsigned int i(0); i<features.size(); i++){
        Eigen::Vector3d featureDir((*features[i]->getViewpoint()->getOrientation()).transpose()*(*features[i]->getDirection()));
        features[i]->setRadius(featureDir.dot(position-(*features[i]->getViewpoint()->getPosition())));
    }
}

bool Structure::computeFilter(double dispTolerence, double triTolerence){
    double localDisp(0.);
    double maxDisp(0.);
    for(unsigned int i(0); i<features.size(); i++){
        Eigen::Vector3d optimalDir(position-(*features[i]->getViewpoint()->getPosition()));
        Eigen::Vector3d featureDir((*features[i]->getViewpoint()->getOrientation()).transpose()*(*features[i]->getDirection()));
        localDisp=acos(optimalDir.dot(featureDir)/optimalDir.norm());
        if ( localDisp>maxDisp ) maxDisp=localDisp;
    }

    if ( maxDisp > dispTolerence ) return false;

    double localAngle(0.);
    double maxAngle(0.);

    for(unsigned int i(0); i<features.size(); i++){
        Eigen::Vector3d iDirection((*features[i]->getViewpoint()->getOrientation()).transpose()*(*features[i]->getDirection()));
        for(unsigned int j(i+1); j<features.size(); j++){
            Eigen::Vector3d jDirection((*features[j]->getViewpoint()->getOrientation()).transpose()*(*features[j]->getDirection()));
            localAngle=acos(iDirection.dot(jDirection));
            if ( localAngle>maxAngle ) maxAngle=localAngle;

        }
    }
    if ( maxAngle<triTolerence ) return false; else return true;
}

