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

unsigned int Structure::getFeaturesCount(){
    return features.size();
}

bool Structure::getActiveStructure(long lastViewpointIndex){
    for(auto & element: features){
        if(element->getViewpoint()->getIndex()==lastViewpointIndex){
            return true;
        }
    }
    return false;
}

void Structure::setFeaturesState(){
    for(auto & element: features){
        element->setStructurePtr(NULL);
        //element->setState(false);
    }
}

void Structure::addFeature(Feature * feature){
    feature->reset();
    feature->setStructurePtr(this);
    features.push_back(feature);
}

void Structure::computeModel(){
    for(auto & element: features){
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
    for(auto & element: features){
        vector=(*element->getViewpoint()->getOrientation())*(*element->getDirection());
        weight=Eigen::Matrix3d::Identity()-vector*vector.transpose();
        vacc+=weight*(*element->getViewpoint()->getPosition());
        wacc+=weight;
    }
    position=wacc.inverse()*vacc;
    flag=true;
}

# ifndef _DEBUG_FLAG
void Structure::computeRadius(long indexLimit){
    Eigen::Vector3d fvector;
    Eigen::Vector3d fposition;
    double radius(0.);
    for(auto & element: features){
        if(element->getViewpoint()->getIndex()>=indexLimit){
            fvector=(*element->getViewpoint()->getOrientation())*(*element->getDirection());
            radius=fvector.dot(position-(*element->getViewpoint()->getPosition()));
            fposition=(*element->getViewpoint()->getPosition())+fvector*radius;
            element->setRadius(radius,(fposition-position).norm());
        }
    }
}
# else
void Structure::computeRadius(){
    Eigen::Vector3d fvector;
    Eigen::Vector3d fposition;
    double radius(0.);
    for(auto & element: features){
        fvector=(*element->getViewpoint()->getOrientation())*(*element->getDirection());
        radius=fvector.dot(position-(*element->getViewpoint()->getPosition()));
        fposition=(*element->getViewpoint()->getPosition())+fvector*radius;
        element->setRadius(radius,(fposition-position).norm());
    }
}
# endif

//void Structure::computeFeaturesState(bool state){
//    for(auto & element: features){
//        element->setState(state);
//    }
//}

bool Structure::computeFilter(double dispFilterSD, double radMean, double radFilterSD){
    for(auto & element: features){
# ifndef _DEBUG_FLAG
        //if (element->getRadius()<0.){
        //    return false;
        //}
# else
        if (element->getRadius()<0.){
            return false;
        }
        //if (element->getDisparity()>dispFilterSD) {
        //    return false;
        //}
        if(fabs(element->getRadius()-radMean)>radFilterSD){
            return false;
        }
# endif
    }
    return true;
}

# ifndef _DEBUG_FLAG
void Structure::extrapolate(){
    return;
}
# else
void Structure::extrapolate(){
    if (flag==false){
        computeOptimalPosition();
    }
    computeRadius();
}
# endif

