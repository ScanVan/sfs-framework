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

# include "framework-structure.hpp"

Eigen::Vector3d * Structure::getPosition(){
    return &position;
}

bool Structure::getOptimised(){
    return optimised;
}

bool Structure::getFiltered(){
    return filtered;
}

int Structure::getFeaturesCount(){
    return features.size();
}

bool Structure::getHasLastViewpoint(int lastViewpointIndex){
    if(features.back()->getViewpoint()->getIndex()==lastViewpointIndex){
        return true;
    }else{
        return false;
    }
}

bool Structure::getHasScale(unsigned int configGroup){
    if(features.size()>=configGroup){
        return true;
    }else{
        return false;
    }
}

bool Structure::getHasHead(unsigned int configGroup, unsigned int lastViewpoint){
    if(features.size()<configGroup){
        return false;
    }
    for(unsigned int i(0); i<configGroup; i++){
        if(features[features.size()-(i+1)]->getViewpoint()->getIndex()!=(lastViewpoint-i)){
            return false;
        }
    }
    return true;
}

cv::Vec3b Structure::getColor(){
    Eigen::Vector3f accum(Eigen::Vector3f::Zero());
    cv::Vec3b color;
    for(auto feature: features){
        color=feature->getColor();
        accum(0)+=color[0];
        accum(1)+=color[1];
        accum(2)+=color[2];
    }
    color[0]=round(accum(0)/features.size());
    color[1]=round(accum(1)/features.size());
    color[2]=round(accum(2)/features.size());
    return color;
}

void Structure::setReset(){
    for(auto & feature: features){
        feature->reset();
    }
    optimised=false;
}

void Structure::setFeaturesState(){
    for(auto & element: features){
        element->setStructurePtr(NULL);
    }
}

void Structure::addFeature(Feature * feature){
    feature->reset();
    feature->setStructurePtr(this);
    features.push_back(feature);
}

void Structure::sortFeatures(){
    int detectSmallest(0), candidateSmallest(0), pushIndex(0);
    std::vector<Feature*> unsorted(features);
    for(unsigned int i(0); i<features.size(); i++){
        detectSmallest=INT_MAX;
        for(unsigned int j(0); j<unsorted.size(); j++){
            if(unsorted[j]!=NULL){
                candidateSmallest=unsorted[j]->getViewpoint()->getIndex();
                if(candidateSmallest<detectSmallest){
                    detectSmallest=candidateSmallest;
                    pushIndex=j;
                }
            }
        }
        features[i]=unsorted[pushIndex];
        unsorted[pushIndex]=NULL;
    }
}

void Structure::computeModel(){
    for(auto & element: features){
        element->computeModel();
    }
}

void Structure::computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms){
    int previousIndex(0);
    for(unsigned int i(1); i<features.size(); i++){
        previousIndex=features[i-1]->getViewpoint()->getIndex();
        if((features[i]->getViewpoint()->getIndex()-previousIndex)==1){
            transforms[previousIndex]->pushCentroid(features[i-1]->getModel(),features[i]->getModel());
        }
    }
}

void Structure::computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms){
    int previousIndex(0);
    for(unsigned int i(1); i<features.size(); i++){
        previousIndex=features[i-1]->getViewpoint()->getIndex();
        if((features[i]->getViewpoint()->getIndex()-previousIndex)==1){
            transforms[previousIndex]->pushCorrelation(features[i-1]->getModel(), features[i]->getModel());
        }
    }
}

void Structure::computeOriented(unsigned int headStart){
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=headStart){
            feature->computeOriented(feature->getViewpoint()->getOrientation());
        }
    }
}

void Structure::computeOptimalPosition(unsigned int headStart){
    Eigen::Vector3d accum(Eigen::Vector3d::Zero());
    Eigen::Vector3d vectorL(Eigen::Vector3d::Zero());
    double dotAB(0.);
    double dotAL(0.);
    double dotBL(0.);
    double norm(0.);
    double rada(0.);
    double radb(0.);
    unsigned int count(0);
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getViewpoint()->getIndex()<headStart) continue;
        for(unsigned int j(i+1); j<features.size(); j++){
            if(features[j]->getViewpoint()->getIndex()<headStart) continue;
            dotAB=features[i]->getModel()->dot(*features[j]->getModel());
            vectorL=(*features[j]->getViewpoint()->getPosition())-(*features[i]->getViewpoint()->getPosition());
            dotAL=features[i]->getModel()->dot(vectorL);
            dotBL=features[j]->getModel()->dot(vectorL);
            norm=1.-(dotAB*dotAB);
            rada=(-dotAB*dotBL+dotAL)/norm;
            radb=(+dotAB*dotAL-dotBL)/norm;

            //if ((1.-fabs(dotAB))>0.087266){
                accum+=0.5*(
                (*features[i]->getViewpoint()->getPosition())+rada*(*features[i]->getModel())
                +
                (*features[j]->getViewpoint()->getPosition())+radb*(*features[j]->getModel())
                );
                count++;
            //}
        }
    }
    position=accum/count;
    optimised=true;
}

void Structure::computeRadius(unsigned int headStart){
    Eigen::Vector3d fposition;
    double radius(0.);
    for(auto & element: features){
        if(element->getViewpoint()->getIndex()>=headStart){
            radius=(*element->getModel()).dot(position-(*element->getViewpoint()->getPosition()));
            fposition=(*element->getViewpoint()->getPosition())+(*element->getModel())*radius;
            element->setRadius(radius,(fposition-position).norm());
        }
    }
}

unsigned int Structure::computeDisparityMean(double * const meanValue,unsigned int headStart){
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=headStart){
            (*meanValue)+=feature->getDisparity();
        }
    }
    return features.size();
}

void Structure::computeDisparityStd(double * const stdValue, double const meanValue, unsigned int headStart){
    double component(0);
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=headStart){
            component=feature->getDisparity()-meanValue;
            (*stdValue)+=component*component;
        }
    }
}

void Structure::computeDisparityMax(double * const maxValue, unsigned int headStart){
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=headStart){
            if(feature->getDisparity()>(*maxValue)){
                (*maxValue)=feature->getDisparity();
            }
        }
    }
}

void Structure::filterRadialRange(double lowClamp, double highClamp,unsigned int headStart){
    unsigned int index(0);
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getViewpoint()->getIndex()>=headStart){
            if((features[i]->getRadius()<lowClamp)||(features[i]->getRadius()>highClamp)){
                features[i]->setStructurePtr(NULL);
            }else{
                if(index!=i) features[index]=features[i];
                index ++;
            }
        }else{
            if(index!=i) features[index]=features[i];
            index ++;
        }
    }
    filterStatus=0;
    if(index<features.size()){
        features.resize(index);
        filterStatus=1;
    }
    if(index<2){
        filtered=false;
        filterStatus=2;
        setFeaturesState();
    }else{
        filtered=true;
    }
}

void Structure::filterDisparity(double limitValue,unsigned int headStart){
    unsigned int index(0);
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getViewpoint()->getIndex()>=headStart){
            if(features[i]->getDisparity()>limitValue){
                features[i]->setStructurePtr(NULL);
            }else{
                if(index!=i) features[index]=features[i];
                index ++;
            }
        }else{
            if(index!=i) features[index]=features[i];
            index ++;
        }
    }
    filterStatus=0;
    if(index<features.size()){
        features.resize(index);
        filterStatus=1;
    }
    if(index<2){
        filtered=false;
        filterStatus=2;
        setFeaturesState();
    }else{
        filtered=true;
    }
}

void Structure::filterExperimental(double minValue){
    double maxDetect(0.), maxCandidate(0.);
    for(unsigned int i(0); i<features.size(); i++){
        Eigen::Vector3d iDirection=*features[i]->getModel();
        for(unsigned int j(i+1); j<features.size(); j++){
            Eigen::Vector3d jDirection=*features[j]->getModel();
            maxCandidate=iDirection.dot(jDirection)/(iDirection.norm()*jDirection.norm());
            if(maxCandidate>maxDetect) maxDetect=maxCandidate;
        }
    }
    if(maxDetect>minValue){
        filtered=false;
        setFeaturesState();
    }else{
        filtered=true;
    }
}
