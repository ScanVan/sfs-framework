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

Database::Database(double initialError, double initialDisparity, double initialRadius, unsigned int initialMatchRange){
    configError=initialError;
    configDisparity=initialDisparity;
    configRadius=initialRadius;
    configMatchRange=initialMatchRange;
}

bool Database::getBootstrap(){
    return viewpoints.size()<DB_LOOP_BOOT_COUNT?true:false;
}

double Database::getPError(){
    return (*viewpoints.back()->getPosition()-*viewpoints.front()->getPosition()).norm();
}

double Database::getDError(){
    return maxValue;
}

bool Database::getCheckError( double const currentError, double const lastError ) {
    if(fabs(currentError-lastError)<configError){
        return true;
    }
    return false;
}

void Database::getTranslationMeanValue(int loopState){
    unsigned int transformRange(transforms.size());
    if(loopState==DB_LOOP_MODE_LAST){
        transformRange--;
    }
    transformMean=0.;
    for(unsigned int i(0); i<transformRange; i++){
        transformMean+=transforms[i]->getTranslation()->norm();
    }
    transformMean/=double(transformRange);

    //transformMean=0.;
    //for(unsigned int i(0); i<transforms.size(); i++){
    //    transformMean+=transforms[i]->getTranslation()->norm();
    //}
    //transformMean/=double(transforms.size());
}

void Database::getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints){
    int localCount = MIN(configMatchRange, viewpoints.size());
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

    // loop on new viewpoint feature
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
        //if(matchCount < 2) continue;
        Structure *structure = NULL;
        switch(structuresCount){
            case 0: {
                // no more needed - direct access to structure
                structure = this->newStructure(newViewpoint);
                structureNewCount++;
            }break;
            case 1: {
                //if(structuresOccurences[0] < 2) continue; //Not good enough
                if(structuresOccurences[0] < 1) continue; //Not good enough
                structure = structures[0];
                structureAggregationCount++;
            }break;
            default: {
                // search for best structure //
                uint32_t detectIdx = 0;
                for ( uint32_t seachMax = 0, searchIdx = 0; searchIdx < structuresCount; searchIdx++ ) {
                    if(structuresOccurences[searchIdx]>seachMax){
                        seachMax=structuresOccurences[searchIdx];
                        detectIdx=searchIdx;
                    }
                }
                if(structuresOccurences[detectIdx] < 1) continue;
                structure = structures[detectIdx];
                structureFusionCount++;
                //continue; // de-activate match fusion
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

void Database::prepareFeature(){

    // parsing structures for features sorting based on relative viewpoint index //
    for(auto & structure: structures){
        structure->sortFeatures();
    }

}

void Database::prepareStructure(){

    // Last viewpoint index
    unsigned int lastViewpoint(viewpoints.size()-1);

    // Continuous indexation
    unsigned int index(0);

    // Copy structures vector
    std::vector<std::shared_ptr<Structure>> unsorted(structures);

    // reset counters
    sortStructTypeA=0;
    sortStructTypeB=0;

    // Type-based detection - Type A
    for(auto & structure: unsorted){
        if(structure->getHasLastViewpoint(lastViewpoint)==true){
            if(structure->getOptimised()==true){
                structures[index++]=structure;
                sortStructTypeA++;
            }

        }
    }

    // Type-based detection - Type B
    for(auto & structure: unsorted){
        if(structure->getHasLastViewpoint(lastViewpoint)==true){
            if(structure->getOptimised()==false){
                structures[index++]=structure;
                sortStructTypeB++;
            }
        }
    }

    // Type-based detection - Type C
    for(auto & structure: unsorted){
        if(structure->getHasLastViewpoint(lastViewpoint)==false){
            structures[index++]=structure;
        }
    }

    // development feature - begin
    if(index!=structures.size()){
        std::cerr << "Fault : " << index << " vs " << structures.size() << std::endl;
    } else {
        std::cerr << "Structure distribution by types : " << sortStructTypeA << ", " << sortStructTypeB << ", " << structures.size() - sortStructTypeA - sortStructTypeB << std::endl;
    }
    // development feature - end

}

void Database::computeModels(int loopState){

    // Active structure upper bound
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;

    }

    // Updtae feature model position
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){
        structures[i]->computeModel();
    }

}

void Database::computeCentroids(int loopState){

    // Active transformation start index
    unsigned int transformationStart(0);

    // Active structure range upper bound
    unsigned int structureRange(structures.size());

    // Check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update active transformation
        transformationStart=transforms.size()-1;

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;

    }

    // Reset active transformations centroid
    # pragma omp parallel for
    for(unsigned int i=transformationStart; i<transforms.size(); i++){
        transforms[i]->resetCentroid();
    }

    // Compute centroid contribution for active structures
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++) {
        if(structures[i]->getHasScale()){
            structures[i]->computeCentroid(transforms);
        }
    }

    // Compute active transformation centroid
    # pragma omp parallel for
    for(unsigned int i=transformationStart; i<transforms.size(); i++){
        transforms[i]->computeCentroid();
    }

}

void Database::computeCorrelations(int loopState){

    // Active transformation start index
    unsigned int transformationStart(0);

    // Active structure range upper bound
    unsigned int structureRange(structures.size());

    // Check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update active transformation
        transformationStart=transforms.size()-1;

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;

    }

    // Reset active transformation correlation matrix
    # pragma omp parallel for
    for(unsigned int i=transformationStart; i<transforms.size(); i++){
        transforms[i]->resetCorrelation();
    }

    // Compute correlation matrix correlation for active structures
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){
        if(structures[i]->getHasScale()){
            structures[i]->computeCorrelation(transforms);
        }
    }

}

void Database::computePoses(int loopState){

    // Active transformation start index
    unsigned int transformationStart(0);

    // Check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update active transformation
        transformationStart=transforms.size()-1;

    }

    // Compute transformation (orientation and translation)
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=transformationStart; i<transforms.size(); i++){
        transforms[i]->computePose();
    }

    // check pipeline state
    //if(loopState==DB_LOOP_MODE_BOOT){

        // Compute translation mean value
        getTranslationMeanValue(loopState);

    //}

    // Renormalise transformations translation
    # pragma omp parallel for 
    //for(unsigned int i=transformationStart; i<transforms.size(); i++){
    for(unsigned int i=0; i<transforms.size(); i++){
        transforms[i]->setTranslationScale(transformMean);
    }

}

void Database::computeFrames(int loopState){

    // Active transformation index
    unsigned int transformationStart(0);

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update active transformation
        transformationStart=transforms.size()-1;

    } else {

        // Assign identity and zero position to first viewpoint
        viewpoints[0]->resetFrame();

    }

    // Compute viewpoint absolute orientation and position
    for(unsigned int i(transformationStart); i<transforms.size(); i++){
        transforms[i]->computeFrame(viewpoints[i].get(),viewpoints[i+1].get());
    }

}

void Database::computeOptimals(long loopState){

    // Active structure range
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update active structure
        structureRange=sortStructTypeA+sortStructTypeB;

    }

    // Compute optimal structure position
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){
        structures[i]->computeOptimalPosition();
    }

}

void Database::computeRadii(long loopState){

    // Active structure upper bound
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;

    }

    // Updtae feature model position
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){
        structures[i]->computeRadius();
    }

}

void Database::computeDisparityStatistics(long loopState){

    // Count increment
    unsigned int countValue(0);

    // Active structure range
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;

    }

    // Compute mean value
    meanValue=0.;
    maxValue=0.;
    for(unsigned int i(0); i<structureRange; i++){
        countValue+=structures[i]->computeDisparityMean(&meanValue);
        structures[i]->computeDisparityMax(&maxValue);
    }
    meanValue/=double(countValue);

    // Compute standard deviation
    stdValue=0.;
    for(unsigned int i(0); i<structureRange; i++){
        structures[i]->computeDisparityStd(&stdValue,meanValue);
    }
    stdValue=std::sqrt(stdValue/(countValue-1));

}

void Database::filterRadialPositivity(int loopState){

    // Differential indexation
    unsigned int index(0);

    // Type-range tracking
    unsigned int trackA(sortStructTypeA);
    unsigned int trackB(sortStructTypeB);

    // Active structure range
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;
        
    }

    // Compute filtering condition
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){

        // Filter structure
        structures[i]->filterRadialPositivity(0.);

    }

    // Filtering loop
    for(unsigned int i(0); i<structures.size(); i++){

        // Check filter flag and passthrough range
        if((i<structureRange)&&(structures[i]->getFiltered()==false)){

            // Track type-range
            if(i<sortStructTypeA){
                trackA --;
            }else if(i<(sortStructTypeA+sortStructTypeB)){
                trackB --;
            }

        } else {

            // Check differential index - move structure
            if(index!=i) structures[index]=structures[i];

            // Update index
            index ++;

        }

    }

    // development feature - begin
    std::cerr << "R:P : " << index << "/" << structures.size() << " (" << trackA << ", " << trackB << ")" << std::endl;
    // development feature - end

    // resize structure vector
    if(index<structures.size()){
        structures.resize(index);
    }

    // Update type-range
    sortStructTypeA=trackA;
    sortStructTypeB=trackB;

}

void Database::filterRadialLimitation(int loopState){

    // Differential indexation
    unsigned int index(0);

    // Type-range tracking
    unsigned int trackA(sortStructTypeA);
    unsigned int trackB(sortStructTypeB);

    // Active structure range
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;
        
    }

    // Compute filtering condition
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){

        // Filter structure
        structures[i]->filterRadialLimitation(transformMean*100.);

    }

    // Filtering loop
    for(unsigned int i(0); i<structures.size(); i++){

        // Check filter flag and passthrough range
        if((i<structureRange)&&(structures[i]->getFiltered()==false)){

            // Track type-range
            if(i<sortStructTypeA){
                trackA --;
            }else if(i<(sortStructTypeA+sortStructTypeB)){
                trackB --;
            }

        } else {

            // Check differential index - move structure
            if(index!=i) structures[index]=structures[i];

            // Update index
            index ++;

        }

    }

    // development feature - begin
    std::cerr << "R:L : " << index << "/" << structures.size() << " (" << trackA << ", " << trackB << ")" << std::endl;
    // development feature - end

    // resize structure vector
    if(index<structures.size()){
        structures.resize(index);
    }

    // Update type-range
    sortStructTypeA=trackA;
    sortStructTypeB=trackB;

}

void Database::filterDisparity(int loopState){

    // Differential indexation
    unsigned int index(0);

    // Type-range tracking
    unsigned int trackA(sortStructTypeA);
    unsigned int trackB(sortStructTypeB);

    // Active structure range
    unsigned int structureRange(structures.size());

    // check pipeline state
    if(loopState==DB_LOOP_MODE_LAST){

        // Update structure range
        structureRange=sortStructTypeA+sortStructTypeB;
        
    }

    // Compute filtering condition
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=0; i<structureRange; i++){

        // Filter structure
        structures[i]->filterDisparity(stdValue*configDisparity);

    }

    // Filtering loop
    for(unsigned int i(0); i<structures.size(); i++){

        // Check filter flag and passthrough range
        if((i<structureRange)&&(structures[i]->getFiltered()==false)){

            // Track type-range
            if(i<sortStructTypeA){
                trackA --;
            }else if(i<(sortStructTypeA+sortStructTypeB)){
                trackB --;
            }

        } else {

            // Check differential index - move structure
            if(index!=i) structures[index]=structures[i];

            // Update index
            index ++;

        }

    }

    // development feature - begin
    std::cerr << "D:D : " << index << "/" << structures.size() << " (" << trackA << ", " << trackB << ")" << std::endl;
    // development feature - end

    // resize structure vector
    if(index<structures.size()){
        structures.resize(index);
    }

    // Update type-range
    sortStructTypeA=trackA;
    sortStructTypeB=trackB;

}

void Database::exportModel(std::string path, unsigned int major){
    std::fstream exportStream;
    exportStream.open(path+"/dev/"+std::to_string(major)+"_structure.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create model exportation file" << std::endl;
        return;
    }
    for(auto & element: structures){
        Eigen::Vector3d * position(element->getPosition());
        if(element->features.size()==2){
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " 255 255 0" << std::endl;
        }else{
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

void Database::exportTransformation(std::string path, unsigned int major){
    std::fstream exportStream;
    exportStream.open(path+"/dev/"+std::to_string(major)+"_transformation.dat",std::ios::out);
    if (exportStream.is_open() == false ){
        std::cerr << "unable to create transformation file" << std::endl;
    }
    for(auto & viewpoint: viewpoints){
        exportStream << viewpoint->uid << " ";
        Eigen::Vector3d * position(viewpoint->getPosition());
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " ";
        Eigen::Matrix3d orientation(viewpoint->orientation);
        exportStream << orientation(0,0) << " " << orientation(0,1) << " " << orientation(0,2) << " ";
        exportStream << orientation(1,0) << " " << orientation(1,1) << " " << orientation(1,2) << " ";
        exportStream << orientation(2,0) << " " << orientation(2,1) << " " << orientation(2,2) << std::endl;
    }
    exportStream.close();
}

//
//  development related features
//

static cv::Point _f2i(Eigen::Vector2f value){
    return cv::Point(value[0],value[1]);
}

struct featureSort
{
    inline bool operator() (const Feature* struct1, const Feature* struct2)
    {
        return (struct1->viewpoint->index < struct2->viewpoint->index);
    }
};

cv::Mat Database::viewpointStructuralImage(Viewpoint *viewpoint, unsigned int structSizeMin){
    cv::RNG rng(12345);
    cv::Rect myROI(0, 0, viewpoint->getImage()->cols, viewpoint->getImage()->rows);
    cv::Mat res(myROI.width,myROI.height, CV_8UC3, cv::Scalar(0,0,0));
    res = *viewpoint->getImage();
    for(unsigned int featureId = 0; featureId < viewpoint->getFeatures()->size(); featureId++){
        auto f = (*viewpoint->getFeatures())[featureId];
        if(!f->structure) continue;
        if(f->structure->features.size() < structSizeMin) continue;
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

        std::vector<Feature*> features = *(f->structure->getFeatures());
        std::sort(features.begin(), features.end(), featureSort());
        for(uint32_t idx = 1;idx < features.size();idx++){
            cv::line(res, _f2i((features)[idx-1]->position),  _f2i((features)[idx]->position), color, 1);
        }
    }

    rng = cv::RNG(12345);
    for(unsigned int featureId = 0; featureId < viewpoint->getFeatures()->size(); featureId++){
        auto f = (*viewpoint->getFeatures())[featureId];
        if(!f->structure) continue;
        if(f->structure->features.size() < structSizeMin) continue;
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

        auto features = f->structure->getFeatures();
//        cv::putText(
//            res,
//            std::to_string(featureId),
//            _f2i((*features)[0]->position) + cv::Point(-5, -5),
//            cv::FONT_HERSHEY_SIMPLEX,
//            0.5,
//            color
//        );
    }
    return res;
}

//Do  cv::waitKey(0); if you want to stop after it.
void Database::_displayViewpointStructures(Viewpoint *viewpoint, unsigned int structSizeMin){
    auto res = viewpointStructuralImage(viewpoint, structSizeMin);

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
        for(auto f : v->features){
            if(f->structure){
                auto sf = &(f->structure->features);
                if(std::find(sf->begin(), sf->end(), f) == sf->end()) throw std::runtime_error("Feature having a structure without that feature");
            }
        }
    }
}

void Database::_sanityCheckStructure(){
    unsigned int lastViewpoint(viewpoints.size()-1);
    for(unsigned int i(0); i<structures.size(); i++){
        if(structures[i]->getFeaturesCount()==2){
            if(structures[i]->getHasLastViewpoint(lastViewpoint)==true){
                if(i>=sortStructTypeA){
                    std::cerr << "Sanity check on structure : fault on type A : " << i << "/" << sortStructTypeA << std::endl;
                }
            }
        }
    }
    for(unsigned int i(0); i<structures.size(); i++){
        if(structures[i]->getFeaturesCount()>2){
            if(structures[i]->getHasLastViewpoint(lastViewpoint)==true){
                if(i>=sortStructTypeA+sortStructTypeB){
                    std::cerr << "Sanity check on structure : fault on type B : " << i << "/" << sortStructTypeA+sortStructTypeB << std::endl;
                }
            }
        }
    }
    for(unsigned int i(0); i<structures.size(); i++){
        if(structures[i]->getHasLastViewpoint(lastViewpoint)==false){
            if(i<sortStructTypeA+sortStructTypeB){
                std::cerr << "Sanity check on structure : fault on type C : " << i << "/" << sortStructTypeA+sortStructTypeB << std::endl;
            }
        }
    }
}

void Database::_sanityCheckFeatureOrder(){
    for(auto & structure: structures){
        for(unsigned int i(1); i<structure->features.size(); i++){
            if(structure->features[i]->getViewpoint()->getIndex()<=structure->features[i-1]->getViewpoint()->getIndex()){
                std::cerr << "Sanity check on feature order" << std::endl;
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
    stream.close();
}

// Note : this function does not respect encapsulation (development function) - need to be removed
void Database::_exportMatchDistribution(std::string path, unsigned int major, std::string type){
    if(viewpoints.size()<DB_LOOP_BOOT_COUNT){
        return;
    }
    std::fstream stream;
    stream.open( path + "/debug/" + std::to_string(major) + "_" + type + ".mat", std::ios::out );
    if(stream.is_open()==false){
        std::cerr << "unable to create match distribution exportation file" << std::endl;
    }
    stream << viewpoints.size() << " -1" << std::endl;
    for(auto & element: structures){
        for(unsigned int i(0); i<element->features.size(); i++){
            for(unsigned int j(0); j<element->features.size(); j++){
                stream << element->features[i]->viewpoint->index+1 << " " << element->features[j]->viewpoint->index+1 << std::endl;
            }
        }
    }
    stream.close();
}

