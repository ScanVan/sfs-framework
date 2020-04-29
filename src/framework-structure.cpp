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

void Structure::computeOriented(){
    for(auto & feature: features){
        feature->computeOriented(feature->getViewpoint()->getOrientation());
    }
}

//compute_intersection

//void Structure::computeOptimalPosition(){
//    Eigen::Vector3d acc(Eigen::Vector3d::Zero());
//    unsigned int count(0);
//
//    for(unsigned int i(0); i<features.size(); i++){
//        for(unsigned int j(i+1); j<features.size(); j++){
//
//            acc+=compute_intersection(
//                features[i]->getViewpoint()->getPosition(),
//                features[i]->getModel(),
//                features[j]->getViewpoint()->getPosition(),
//                features[j]->getModel()
//            );
//            count++;
//
//        }
//    }
//    position=acc/count;
//    optimised=true;
//}

void Structure::computeOptimalPosition(){
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
        for(unsigned int j(i+1); j<features.size(); j++){

            dotAB=features[i]->getModel()->dot(*features[j]->getModel());

            vectorL=(*features[j]->getViewpoint()->getPosition())-(*features[i]->getViewpoint()->getPosition());

            dotAL=features[i]->getModel()->dot(vectorL);
            dotBL=features[j]->getModel()->dot(vectorL);

            norm=1.-(dotAB*dotAB);

            rada=(-dotAB*dotBL+dotAL)/norm;
            radb=(+dotAB*dotAL-dotBL)/norm;

            accum+=0.5*(
            (*features[i]->getViewpoint()->getPosition())+rada*(*features[i]->getModel())
            +
            (*features[j]->getViewpoint()->getPosition())+radb*(*features[j]->getModel())
            );
            count++;

            //accum+=compute_intersection(
            //    features[i]->getViewpoint()->getPosition(),
            //    features[i]->getModel(),
            //    features[j]->getViewpoint()->getPosition(),
            //    features[j]->getModel()
            //);
            //count++;

        }
    }
    position=accum/count;
    optimised=true;
}


//Eigen::Vector3d compute_intersection(Eigen::Vector3d * p1, Eigen::Vector3d * d1, Eigen::Vector3d * p2, Eigen::Vector3d * d2){

//    Eigen::Vector3d d0( *p2 - *p1 );

//    double aa(d1->dot(*d1));
//    double bb(d2->dot(*d2));
//    double ab(d1->dot(*d2));

//    double ac(d1->dot(d0));
//    double bc(d2->dot(d0));

//    double dn(aa*bb-ab*ab);

//    double r1((- ab * bc + ac * bb )/dn);
//    double r2((+ ab * ac - bc * aa )/dn);

//    d0 = 0.5 * ( *p1+(*d1*r1) + *p2+(*d2*r2) );

//    return d0;
//}


//void Structure::computeOptimalPosition(){
//    Eigen::Matrix3d wacc(Eigen::Matrix3d::Zero());
//    Eigen::Vector3d vacc(Eigen::Vector3d::Zero());
//    Eigen::Matrix3d weight;
//    for(auto & element: features){
//        weight=Eigen::Matrix3d::Identity()-(*element->getModel())*(*element->getModel()).transpose();
//        vacc+=weight*(*element->getViewpoint()->getPosition());
//        wacc+=weight;
//    }
//    position=wacc.inverse()*vacc;
//    optimised=true;
//}

void Structure::computeRadius(){
    Eigen::Vector3d fposition;
    double radius(0.);
    for(auto & element: features){
        radius=(*element->getModel()).dot(position-(*element->getViewpoint()->getPosition()));
        fposition=(*element->getViewpoint()->getPosition())+(*element->getModel())*radius;
        element->setRadius(radius,(fposition-position).norm());
    }
}

unsigned int Structure::computeDisparityMean(double * const meanValue){
    for(auto & feature: features){
        (*meanValue)+=feature->getDisparity();
    }
    return features.size();
}

void Structure::computeDisparityStd(double * const stdValue, double const meanValue){
    double component(0);
    for(auto & feature: features){
        component=feature->getDisparity()-meanValue;
        (*stdValue)+=component*component;
    }
}

void Structure::computeDisparityMax(double * const maxValue){
    for(auto & feature: features){
        if(feature->getDisparity()>(*maxValue)){
            (*maxValue)=feature->getDisparity();
        }
    }
}

void Structure::filterRadialRange(double lowClamp, double highClamp){
    unsigned int index(0);
    for(unsigned int i(0); i<features.size(); i++){
        if((features[i]->getRadius()<lowClamp)||(features[i]->getRadius()>highClamp)){
            features[i]->setStructurePtr(NULL);
        }else{
            if(index!=i) features[index]=features[i];
            index ++;
        }
    }
    if(index<features.size()){
        features.resize(index);
    }
    if(index<2){
        filtered=false;
        setFeaturesState();
    }else{
        filtered=true;
    }
}

void Structure::filterDisparity(double limitValue){
    unsigned int index(0);
    for(unsigned int i(0); i<features.size(); i++){
        if(features[i]->getDisparity()>limitValue){
            features[i]->setStructurePtr(NULL);
        }else{
            if(index!=i) features[index]=features[i];
            index ++;
        }
    }
    if(index<features.size()){
        features.resize(index);
    }
    if(index<2){
        filtered=false;
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
