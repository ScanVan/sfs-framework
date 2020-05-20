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

#include "framework-structure.hpp"

bool Structure::getHasScale(unsigned int scaleGroup){

    // Check if structure broadcast the scale information
    if(features.size()>=scaleGroup){
        return true;
    }else{
        return false;
    }

}

Eigen::Vector3d * Structure::getPosition(){

    // Return structure position
    return &position;

}

unsigned int Structure::getState(){

    // Return structure state
    return state;

}

cv::Vec3b Structure::getColor(){

    // Acumulation vector
    Eigen::Vector3f accum(Eigen::Vector3f::Zero());

    // Returned color object
    cv::Vec3b color;

    // Accumulate colors
    for(auto feature: features){
        color=feature->getColor();
        accum(0)+=color[0];
        accum(1)+=color[1];
        accum(2)+=color[2];
    }

    // Compute color mean
    color[0]=round(accum(0)/features.size());
    color[1]=round(accum(1)/features.size());
    color[2]=round(accum(2)/features.size());

    // Return structure color
    return color;
}

void Structure::setReset(){

    // Reset feature radius and disparity
    for(auto & feature: features){
        feature->reset();
    }

}

void Structure::addFeature(Feature * feature){

    // Reset feature radius and disparity
    feature->reset();

    // Assign structure pointer to feature
    feature->setStructurePtr(this);

    // Add feature to structure
    features.push_back(feature);

}

void Structure::sortFeatures(){
    
    // Re-indexation
    int detectSmallest(0), candidateSmallest(0), pushIndex(0);

    // Features vector copy
    std::vector<Feature*> unsorted(features);

    // Sorting features
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

void Structure::computeState(unsigned int scaleGroup, unsigned int highViewpoint){

    // Check if structure has last viewpoint
    if(features.back()->getViewpoint()->getIndex()==highViewpoint){

        // Check if structure broadcast the scale information
        if(features.size()>=scaleGroup){

            // Assume state
            state=STRUCTURE_PIONER;

            // Check if structure has continuous sequence of feature from last viewpoint
            for(unsigned int i(0); i<scaleGroup; i++){

                // Detect continous sequence
                if(features[features.size()-(i+1)]->getViewpoint()->getIndex()!=(highViewpoint-i)){
                    state=STRUCTURE_NORMAL;
                    return;
                }

            }

        }else{

            // Assign state
            state=STRUCTURE_NORMAL;

        }

    }else{

        // Assign state
        state=STRUCTURE_NORMAL;

    }

}

void Structure::computeModel(){

    // Compute features model - According to their respective viewpoint
    for(auto & feature: features){
        feature->computeModel();
    }

}

void Structure::computeCentroid(std::vector<std::shared_ptr<Transform>> & transforms, unsigned int lowViewpoint){

    // Low index
    unsigned int index(0);

    // Detect and add features contribution to centroid
    for(unsigned int i(features.size()-1); i>0; i--){
        if((index=features[i-1]->getViewpoint()->getIndex())>=lowViewpoint){
            if((features[i]->getViewpoint()->getIndex()-index)==1){
                transforms[index]->pushCentroid(features[i-1]->getModel(),features[i]->getModel());
            }
        }
    }

}

void Structure::computeCorrelation(std::vector<std::shared_ptr<Transform>> & transforms, unsigned int lowViewpoint){

    // Low index
    unsigned int index(0);

    // Detect and add features contribution to correlation matrix
    for(unsigned int i(features.size()-1); i>0; i--){
        if((index=features[i-1]->getViewpoint()->getIndex())>=lowViewpoint){
            if((features[i]->getViewpoint()->getIndex()-index)==1){
                transforms[index]->pushCorrelation(features[i-1]->getModel(), features[i]->getModel());
            }
        }
    }

}


void Structure::computeOriented(unsigned int lowViewpoint){

    // Compute oriented features - According to absolute frame
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=lowViewpoint){
            feature->computeOriented(feature->getViewpoint()->getOrientation());
        }
    }

}

void Structure::computeOptimalPosition(unsigned int lowViewpoint){

    // Optimal intersection position
    Eigen::Vector3d optimal(Eigen::Vector3d::Zero());

    // Baseline vector
    Eigen::Vector3d baseline;

    // Dot products
    double dotij(0.);
    double dotib(0.);
    double dotjb(0.);

    // Norm value
    double norm(0.);

    // Intersection count
    unsigned int count(0);

    // Parsing features
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getViewpoint()->getIndex()>=lowViewpoint){
            for(unsigned int j(i+1); j<features.size(); j++){
                if(features[j]->getViewpoint()->getIndex()>=lowViewpoint){

                    // Compute baseline
                    baseline=(*features[j]->getViewpoint()->getPosition())-(*features[i]->getViewpoint()->getPosition());
                    
                    // Compute dot products
                    dotij=features[i]->getModel()->dot(*features[j]->getModel());
                    dotib=features[i]->getModel()->dot(baseline);
                    dotjb=features[j]->getModel()->dot(baseline);

                    // Compute norm
                    norm=1.-(dotij*dotij);
    
                    // Accumulate optimal position
                    optimal+=0.5*(
                    (*features[i]->getViewpoint()->getPosition())+((-dotij*dotjb+dotib)/norm)*(*features[i]->getModel()) +
                    (*features[j]->getViewpoint()->getPosition())+((+dotij*dotib-dotjb)/norm)*(*features[j]->getModel())
                    );

                    // Update count 
                    count++;

                }
            }
        }
    }

    // Compute optimal position
    position=optimal/double(count);

}

void Structure::computeRadius(unsigned int lowViewpoint){
    
    // Feature position in absolute frame
    Eigen::Vector3d vector;

    // Radius value
    double radius(0.);

    // Compute feature radius accroding to structure position in absolute frame
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=lowViewpoint){
            vector=position-(*feature->getViewpoint()->getPosition());
            radius=(*feature->getModel()).dot(vector);
            feature->setRadius(radius,(vector-(*feature->getModel())*radius).norm());
        }
    }

}

unsigned int Structure::computeDisparityMean(double * const meanValue,unsigned int lowViewpoint){

    // Contribution count to mean
    int count(0);

    // Accumulate disparity values
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=lowViewpoint){
            (*meanValue)+=feature->getDisparity();
            count++;
        }
    }

    // Return contribution count to mean
    return count;

}

void Structure::computeDisparityStd(double * const stdValue, double const meanValue, unsigned int lowViewpoint){

    // Standard deviation component
    double component(0);

    // Compute standard deviation contribution
    for(auto & feature: features){
        if(feature->getViewpoint()->getIndex()>=lowViewpoint){
            component=feature->getDisparity()-meanValue;
            (*stdValue)+=component*component;
        }
    }

}

void Structure::filterRadialRange(double lowClamp, double highClamp,unsigned int lowViewpoint){

    // Re-sampling index
    unsigned int index(0);

    // Filtering structure features
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getViewpoint()->getIndex()>=lowViewpoint){

            // Filter condition
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

    // Resize feature array and update structure state
    if(index<features.size()){
        filterResize(index);
    }

}

void Structure::filterDisparity(double limitValue,unsigned int lowViewpoint){

    // Re-sampling index
    unsigned int index(0);

    // Filtering structure features
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getViewpoint()->getIndex()>=lowViewpoint){

            // Filter condition
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

    // Resize feature array and update structure state
    if(index<features.size()){
        filterResize(index);
    }

}

void Structure::filterResize(unsigned int resize){

    // Check removal condition
    if(resize<2){
        for(unsigned int i(0); i<resize; i++){
            features[i]->setStructurePtr(NULL);
        }
        features.clear();
        state=STRUCTURE_REMOVE;
    }else{
        features.resize(resize);
        state=STRUCTURE_NORMAL;
    }

}

