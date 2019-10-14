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
    for(auto element: features){
        element->computeModel();
    }
}

void Structure::computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms){
    unsigned int i_index(0), j_index(0);
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
    unsigned int i_index(0), j_index(0);
    for(unsigned int i(0); i<features.size(); i++){
        i_index=features[i]->getViewpoint()->getIndex();
        for(unsigned int j(0); j<features.size(); j++){
            j_index=features[j]->getViewpoint()->getIndex();
            if (i_index-j_index==1){
                transforms[j_index]->pushCorrelation(features[j]->getModel(), features[i]->getModel());
            }
        }
    }
}

void Structure::computeOptimalPosition(){
    Eigen::Matrix3d wacc(Eigen::Matrix3d::Zero());
    Eigen::Vector3d vacc(Eigen::Vector3d::Zero());
    Eigen::Matrix3d weight;
    Eigen::Vector3d vector;
    for(auto element: features){
        vector=(*element->getViewpoint()->getOrientation())*(*element->getDirection());
        weight=Eigen::Matrix3d::Identity()-vector*vector.transpose();
        vacc+=weight*(*element->getViewpoint()->getPosition());
        wacc+=weight;
    }
    position=wacc.inverse()*vacc;
    flag=true;
}

void Structure::computeRadius(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    Eigen::Vector3d fvector;
    Eigen::Vector3d fposition;
    double radius(0.);
    for(auto element: features){
        fvector=(*element->getViewpoint()->getOrientation())*(*element->getDirection());
        radius=fvector.dot(position-(*element->getViewpoint()->getPosition()));
        fposition=(*element->getViewpoint()->getPosition())+fvector*radius;
        element->setRadius(radius,(fposition-position).norm());
    }
}

bool Structure::computeFilter(double dispFilter, double radFilter, double radMean){
    for(auto element: features){
        if (element->getDisparity()>dispFilter){
            return false;
        }
        if (std::fabs(element->getRadius()-radMean)>radFilter){
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

