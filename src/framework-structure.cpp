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

unsigned int Structure::getFeatureCount(){
    return features.size();
}

double Structure::getRadius(unsigned int featureID){
    return features[featureID]->getRadius();
}

double Structure::getDisparity(unsigned int featureID){
    return features[featureID]->getDisparity();
}

void Structure::computeModel(){
    for(unsigned int i(0); i<features.size(); i++){
        features[i]->computeModel();
    }
}

void Structure::computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms){
    int i_index(0), j_index(0);
    for(unsigned int i(0); i<features.size(); i++){
        i_index=features[i]->getViewpoint()->getIndex();
        for(unsigned int j(0); j<features.size(); j++){
            j_index=features[j]->getViewpoint()->getIndex();
            if (i_index-j_index==1){
                transforms[j_index]->pushCentroid(features[j]->getModel(),features[i]->getModel());
            }
        }
    }
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
        Eigen::Vector3d dirVector((*features[i]->getViewpoint()->getOrientation())*(*features[i]->getDirection()));
        Eigen::Matrix3d weight(Eigen::Matrix3d::Identity()-dirVector*dirVector.transpose());
        macc+=weight;
        vacc+=weight*(*features[i]->getViewpoint()->getPosition());
    }
    position=macc.inverse()*vacc;
    flag=true;
}

void Structure::computeRadius(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    for(unsigned int i(0); i<features.size(); i++){
        Eigen::Vector3d featureDir((*features[i]->getViewpoint()->getOrientation())*(*features[i]->getDirection()));
        Eigen::Vector3d optimalDir(position-(*features[i]->getViewpoint()->getPosition()));
        double radius(featureDir.dot(position-(*features[i]->getViewpoint()->getPosition())));
        features[i]->setRadius(radius);
        Eigen::Vector3d fposition((*features[i]->getViewpoint()->getPosition())+featureDir*radius);
        features[i]->setDisparity((fposition-position).norm());
    }
}

bool Structure::computeFilter(double dispSD, double radMean, double radSD, double dispTolerence, double radTolerence){
    double dispFilter(dispSD*dispTolerence);
    double radFilter(radSD*radTolerence);
    for(unsigned int i(0); i<features.size(); i++){
        if (features[i]->getDisparity()>dispFilter){
            return false;
        }
        if (std::fabs(features[i]->getRadius()-radMean)>radFilter){
            return false;
        }
    }
    return true;
}

void Structure::extrapolate(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    if (flag==false){
        computeOptimalPosition();
    }
    computeRadius(viewpoints);
}

