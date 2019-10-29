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

#include "framework-database.hpp"

Database::Database(
    double initialError,
    unsigned long initialStructure,
    double initialDisparity,
    double initialRadius
){
    configError=initialError;
    configStructure=initialStructure;
    configDisparity=initialDisparity;
    configRadius=initialRadius;
}

bool Database::getBootstrap(){
    return viewpoints.size()<configStructure?true:false;
}

double Database::getConfigError(){
    return configError;
}

# ifndef _DEBUG_FLAG
double Database::getError(){
    return (viewpoints.back()->position - viewpoints.front()->position).norm();
}
# else
double Database::getError(){
    return dispMean;
}
# endif

# ifndef _DEBUG_FLAG
double Database::getTranslationMeanValue(){
    double meanValue(0.);
    for(auto & element: transforms){
        meanValue+=element->getTranslation()->norm();
    }
    return meanValue/double(transforms.size());
}
# endif

void Database::getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints){
    int localCount = MIN(5, viewpoints.size());
    for(auto i = viewpoints.end()-localCount;i != viewpoints.end(); ++i){
        localViewpoints->push_back(*i);
    }
}

void Database::addViewpoint(std::shared_ptr<Viewpoint> viewpoint){
    if(viewpoint->getIndex() > 0) transforms.push_back(std::make_shared<Transform>());
    viewpoints.push_back(viewpoint);
}

void Database::aggregate(std::vector<std::shared_ptr<Viewpoint>> *localViewpoints, Viewpoint *newViewpoint, uint32_t *correlations){
    uint32_t localViewpointsCount = localViewpoints->size();
    Structure** structures = new Structure*[localViewpointsCount];
    uint32_t* structuresOccurences = new uint32_t[localViewpointsCount];
    uint32_t structureNewCount = 0;
    uint32_t structureAggregationCount = 0;
    uint32_t structureFusionCount = 0;
    uint32_t *viewpointsUsage = new uint32_t[viewpoints.size() + 1];
    memset(viewpointsUsage, -1, (viewpoints.size() + 1)*sizeof(uint32_t));
    newViewpoint->setIndex(viewpoints.size());
    for(uint32_t queryIdx = 0;queryIdx < newViewpoint->features.size(); queryIdx++){
        uint32_t *correlationsPtr = correlations + queryIdx*localViewpointsCount; //Used to iterate over the given lines

        uint32_t structuresCount = 0;
        uint32_t matchCount = 0;

        //Collect all the existing structures of the matches and count their occurences
        for(uint32_t localIdx = 0;localIdx < localViewpointsCount;localIdx++){
            uint32_t trainIdx = correlationsPtr[localIdx];
            if(trainIdx != 0xFFFFFFFF){
                matchCount++;
                auto localFeature = (*localViewpoints)[localIdx]->getFeatureFromCvIndex(trainIdx);
                auto localStructure = localFeature->structure;
                if(localStructure){
                    uint32_t cacheIdx;
                    for(cacheIdx = 0;cacheIdx < structuresCount; cacheIdx++){
                        if(structures[cacheIdx] == localStructure){
                            structuresOccurences[cacheIdx]++;
                            break;
                        }
                    }
                    if(cacheIdx == structuresCount){ //No cache hit
                        structures[cacheIdx] = localStructure;
                        structuresOccurences[cacheIdx] = 1;
                        structuresCount++;
                    }
                }
            }
        }

        //Figure out which structure will be used to integrate the newViewpoint feature
        if(matchCount == 0) continue; //No match => no integration
        Structure *structure = NULL;
        switch(structuresCount){
            case 0: {
                // no more needed - direct access to structure
                structure = this->newStructure();
                structureNewCount++;
            }break;
            case 1: {
                if(structuresOccurences[0] < 2) continue; //Not good enough
                structure = structures[0];
                structureAggregationCount++;
            }break;
            default: {
                structureFusionCount++;
                continue;
            }break;
        }

        //Populate pre-existing viewpoint usage from the structure
        for(auto f : structure->features){
            viewpointsUsage[f->viewpoint->index] = queryIdx;
        }

        //Integrate all orphan feature into the common structure
        for(uint32_t localIdx = 0;localIdx < localViewpointsCount;localIdx++){
            uint32_t trainIdx = correlationsPtr[localIdx];
            if(trainIdx != 0xFFFFFFFF){
                auto localFeature = (*localViewpoints)[localIdx]->getFeatureFromCvIndex(trainIdx);
                auto viewpointId = (*localViewpoints)[localIdx]->index;
                if(!localFeature->structure && viewpointsUsage[viewpointId] != queryIdx){
                    viewpointsUsage[viewpointId] = queryIdx;
                    structure->addFeature((*localViewpoints)[localIdx]->getFeatureFromCvIndex(trainIdx));
                }
            }
        }

        auto newFeature = newViewpoint->getFeatureFromCvIndex(queryIdx);
        if(newFeature->structure) throw std::runtime_error("New feature already had a structure");
        if(viewpointsUsage[viewpoints.size()] != queryIdx){
            structure->addFeature(newFeature);
        }
    }
    delete[] viewpointsUsage;
    delete[] structures;
    delete[] structuresOccurences;
    std::cout << "structureNewCount=" << structureNewCount << " structureAggregationCount=" << structureAggregationCount << " structureFusionCount=" << structureFusionCount << std::endl;

}

/* only needed for computed radius : \
   0 : all
   1 : only last viewpoint
   2 : only head viewpoints
   3 : all
*/

void Database::computeModels(){
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeModel();
        }
    }
}

void Database::computeCorrelations(){
    for(auto & element: transforms){
        element->resetCorrelation();
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeCorrelation(transforms);
        }
    }
}

void Database::computeCentroids(){
    for(auto & element: transforms){
        element->resetCentroid();
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeCentroid(transforms);
        }
    }
    for(auto & element: transforms){
        element->computeCentroid();
    }
}

# ifndef _DEBUG_FLAG
void Database::computePoses(long loopState){
    double normalValue(0.);
    int mode(0);
    if (loopState==1){
        mode=transforms.size()-1;
    }
    for(unsigned int i(mode); i<transforms.size(); i++){
        transforms[i]->computePose(viewpoints[i].get(),viewpoints[i+1].get());
    }
    normalValue=getTranslationMeanValue();
    for(auto & element: transforms){
        element->setTranslationScale(normalValue);
    }
}
# else
void Database::computePoses(){
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->computePose(viewpoints[i].get(),viewpoints[i+1].get());
    }
    double normalFactor(0.);
    for(unsigned int i(0); i<MIN(10,transforms.size()); i++){
        normalFactor+=transforms[i]->getTranslation()->norm();
    }
    normalFactor/=MIN(10,transforms.size());
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->setTranslationScale(normalFactor);
    }
}
# endif

void Database::computeFrames(){
    viewpoints[0]->resetFrame();
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->computeFrame(viewpoints[i].get(),viewpoints[i+1].get());
    }
}

# ifndef _DEBUG_FLAG
void Database::computeOptimals(long loopState){
    if((loopState==2)||(loopState==0)){
        for(auto & element: structures){
            if(element->getFeaturesCount()>=configStructure){
                element->computeOptimalPosition();
            }
        }
    }
}
# else
void Database::computeOptimals(){
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeOptimalPosition();
        }
    }
}
# endif

# ifndef _DEBUG_FLAG
void Database::computeRadii(long loopState){
    long mode(0);
    if(loopState==1){
        mode=viewpoints.size()-1;
    }
    for(auto & element: structures){
        if(element->flag==true){ // not needed if configStruc==2 : to be checked
        if(element->getFeaturesCount()>=configStructure){
            element->computeRadius(mode);
        }
        }
    }
}
# else
void Database::computeRadii(){
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeRadius();
        }
    }
}
# endif

# ifndef _DEBUG_FLAG
# else
void Database::computeStatistics(){
    unsigned long count(0);
    double component;
    dispMean=0.;
    radMean=0.;
    for(auto & element: structures){
        if(element->flag==true){
        if(element->getFeaturesCount()>=configStructure){
            for(auto & f: element->features){
                dispMean+=f->disparity;
                radMean+=f->radius;
            }
            count+=element->getFeaturesCount();
        }
        }
    }
    dispMean/=double(count);
    radMean/=double(count);

    dispSD=0.;
    radSD=0.;
    for(auto & element: structures){
        if(element->flag==true){
        if(element->getFeaturesCount()>=configStructure){
            for(auto & f: element->features){
                component=f->disparity-dispMean;
                dispSD+=component*component;
                component=f->radius-radMean;
                radSD+=component*component;
            }
        }
        }
    }
    dispSD=std::sqrt(dispSD/double(count-1))*configDisparity;
    radSD=std::sqrt(radSD/double(count-1))*configRadius;
}
# endif

//Issue index of following elements will be modifed, can't be use in computeFilter as this
//void Database::deleteAndUnlinkStructure(int i){
//    for(auto f : *structures[i]->getFeatures()){
//        f->structure = NULL;
//    }
//    std::swap(structures[i],structures[structures.size()-1]);
//  structures.resize(structures.size()-1);
//}

# ifndef _DEBUG_FLAG
void Database::computeFilters(){
    return;
    computeFiltersEliminate(&Feature::getRadius, &Structure::computeFilterDownClamp,1.0,0.0);
    computeFiltersStatistics(&Feature::getRadius);
    std::cerr << meanValue << " " << stdValue << std::endl;
    computeFiltersEliminate(&Feature::getRadius, &Structure::computeFilterStatistics,meanValue,stdValue*configRadius);
}

/* this member should to be private */
void Database::computeFiltersStatistics(double(Feature::*getValue)()){
    unsigned long countValue(0);
    double componentValue(0.);
    meanValue=0.;
    for(auto & element: structures){
        for(auto & feature: element->features){
            meanValue+=(feature->*getValue)();
            countValue++;
        }
    }
    meanValue/=double(countValue);
    stdValue=0.;
    for(auto & element: structures){
        for(auto & feature: element->features){
            componentValue=(feature->*getValue)()-meanValue;
            stdValue+=componentValue*componentValue;
        }
    }
    stdValue=std::sqrt(stdValue/(countValue-1));
}

/* this member should to be private */
void Database::computeFiltersEliminate(double(Feature::*getValue)(), bool (Structure::*filterMethod)(double(Feature::*)(),double,double), double filteringValue, double dummy){
    unsigned int i(0);
    unsigned int j(structures.size());
    while (i < j){
        //if(structures[i]->getActiveStructure(viewpoints.size()-1)==true){
            if(structures[i]->getFeaturesCount()>=configStructure){
                if ((structures[i].get()->*filterMethod)(getValue, filteringValue, dummy)==false){
                    structures[i]->setFeaturesState();
                    std::swap(structures[i],structures[--j]);
                } else {
                    i++;
                }
            }else{i++;}
        //}else{i++;}
    }
    structures.resize(j);
}
# else
void Database::computeFilters(){
    unsigned int i(0);
    unsigned int j(structures.size());
    while (i < j){
        if(structures[i]->getActiveStructure(viewpoints.size()-1)==true){
            if(structures[i]->getFeaturesCount()>=configStructure){
                if (structures[i]->computeFilter(dispSD,radMean,radSD)==false){
                    structures[i]->setFeaturesState();
                    std::swap(structures[i],structures[--j]);
                } else {
                    i++;
                }
            }else{i++;}
        }else{i++;}
    }
    structures.resize(j);
}
# endif

/* Note : called before viewpoint push on stack */
# ifndef _DEBUG_FLAG
void Database::extrapolateViewpoint(Viewpoint * pushedViewpoint){
    pushedViewpoint->resetFrame();
}
# else
void Database::extrapolateViewpoint(Viewpoint * pushedViewpoint){
    auto viewpointCount(viewpoints.size());
    if(viewpointCount<configStructure){
        pushedViewpoint->resetFrame();
    }else{
        Eigen::Matrix3d prevRotation((*transforms[viewpointCount-2]->getRotation()).transpose());
        Eigen::Vector3d prevTranslation(prevRotation*(*transforms[viewpointCount-2]->getTranslation()));
        pushedViewpoint->setPose(
            (*viewpoints[viewpointCount-1]->getOrientation())*prevRotation,
            (*viewpoints[viewpointCount-1]->getPosition())-prevTranslation
        );
    }
}
# endif

/* Note : called after viewpoint push on stack */
# ifndef _DEBUG_FLAG
void Database::extrapolateStructure(){
    return;
    if(viewpoints.size()>configStructure){
        for(auto & element: structures){
            element->extrapolate();
        }
    }
}
# else
void Database::extrapolateStructure(){
    if(viewpoints.size()>configStructure){
        for(auto & element: structures){
            element->extrapolate();
        }
    }
}
# endif

void Database::exportModel(std::string path, unsigned int major){
    std::fstream exportStream;
    exportStream.open(path+"/dev/"+std::to_string(major)+"_structure.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create model exportation file" << std::endl;
        return;
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            Eigen::Vector3d * position(element->getPosition());
            exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " 255 0 0" << std::endl;
        }
    }
    exportStream.close();
}

void Database::exportOdometry(std::string path, unsigned int major){
    std::fstream exportStream;
    exportStream.open(path+"/dev/"+std::to_string(major)+"_odometry.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create odometry exportation file" << std::endl;
        return;
    }
    for(auto & element: viewpoints){
        Eigen::Vector3d * position(element->getPosition());
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " 255 255 255" << std::endl;
    }
    exportStream.close();
}

//
//  development related features
//

static cv::Point _f2i(Eigen::Vector2f value){
    return cv::Point(value[0],value[1]);
}

//Do  cv::waitKey(0); if you want to stop after it.
void Database::_displayViewpointStructures(Viewpoint *viewpoint, unsigned int structSizeMin){
    cv::RNG rng(12345);
    cv::Rect myROI(0, 0, viewpoint->getImage()->cols, viewpoint->getImage()->rows);
    cv::Mat res(myROI.width,myROI.height, CV_8UC3, cv::Scalar(0,0,0));
    res = *viewpoint->getImage();
    for(auto f : *viewpoint->getFeatures()){
        if(!f.structure) continue;
        if(f.structure->features.size() < structSizeMin) continue;
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

        auto features = f.structure->getFeatures();
        for(uint32_t idx = 1;idx < features->size();idx++){
            cv::line(res, _f2i((*features)[idx-1]->position),  _f2i((*features)[idx]->position), color, 2);
        }
    }

    cv::namedWindow( "miaou", cv::WINDOW_KEEPRATIO );
    imshow( "miaou", res);
}

void Database::_sanityCheck(bool inliner){
    //How many structure have a given size (size is the index)
    uint32_t *structureSizes = new uint32_t[viewpoints.size()+1];
    memset(structureSizes, 0, (viewpoints.size()+1)*sizeof(uint32_t));

    //Set<Viewpoint> to identify viewpoint duplication in structures
    uint32_t *viewpointsUsage = new uint32_t[viewpoints.size()];
    memset(viewpointsUsage, -1, (viewpoints.size())*sizeof(uint32_t));

    for(uint32_t structureId = 0;structureId < structures.size();structureId++){
        auto &s = structures[structureId];
        if(s->features.size() < 2) throw std::runtime_error("Structure with less than two feature");
        structureSizes[s->features.size()]++;
        auto inliner = s->features.front()->inliner;
        for(auto &f : s->features){
            if(f->structure != s.get()) throw std::runtime_error("Structure with a feature which isn't pointing that structure");
            if(inliner) if(f->inliner != inliner) throw std::runtime_error("Inliner issue");
            auto viewpointId = f->viewpoint->index;
            if(viewpointsUsage[viewpointId] == structureId) throw std::runtime_error("Same view point twice in a structure");
            viewpointsUsage[viewpointId] = structureId;
        }
    }
    delete []viewpointsUsage;

    std::cout << "Structure family ";
    for(uint32_t size = 0;size <= viewpoints.size(); size++){
        auto count = structureSizes[size];
        if(count) std::cout << size << "=>" << count << " ";
    }
    std::cout << std::endl;
    delete []structureSizes;

    for(auto v : viewpoints){
        for(auto &f : v->features){
            if(f.structure){
                auto sf = &(f.structure->features);
                if(std::find(sf->begin(), sf->end(), &f) == sf->end()) throw std::runtime_error("Feature having a structure without that feature");
            }
        }
    }
}

// Note : this function does not respect encapsulation (development function) - need to be removed
void Database::_exportState(std::string path, int major, int iter){
    int vpcount(255/viewpoints.size());
    std::fstream stream;
    stream.open( path + "/debug/" + std::to_string(major) + "_" + std::to_string(iter) + ".xyz", std::ios::out );
    if (stream.is_open()==false){
        std::cerr << "unable to create state exportation file" << std::endl;
        return;
    }
    for(auto & element: viewpoints){
        stream << element->position(0) << " "
               << element->position(1) << " "
               << element->position(2) << " 0 0 255" << std::endl;
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            stream << element->position(0) << " "
                   << element->position(1) << " "
                   << element->position(2) << " 255 0 255" << std::endl;
            for(unsigned int j(0); j<element->features.size(); j++){
                Eigen::Matrix3d matrix(*element->features[j]->getViewpoint()->getOrientation());
                Eigen::Vector3d vector(*element->features[j]->getViewpoint()->getPosition());
                Eigen::Vector3d Position(matrix*(element->features[j]->direction*element->features[j]->radius)+vector);
                stream << Position(0) << " "
                       << Position(1) << " "
                       << Position(2) << " 255 " << j*vpcount << " 0" << std::endl;
            }
        }
    }
    stream.close();
}

// Note : this function does not respect encapsulation (development function) - need to be removed
void Database::_exportMatchDistribution(std::string path, unsigned int major, std::string type){
    if(viewpoints.size()<configStructure){
        return;
    }
    std::fstream stream;
    stream.open( path + "/debug/" + std::to_string(major) + "_" + type + ".mat", std::ios::out );
    if(stream.is_open()==false){
        std::cerr << "unable to create match distribution exportation file" << std::endl;
    }
    stream << viewpoints.size() << " -1" << std::endl;
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            for(unsigned int i(0); i<element->features.size(); i++){
                for(unsigned int j(0); j<element->features.size(); j++){
                    stream << element->features[i]->viewpoint->index+1 << " " << element->features[j]->viewpoint->index+1 << std::endl;
                }
            }
        }
    }
    stream.close();
}

