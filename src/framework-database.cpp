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

#include "framework-database.hpp"

//
//  Framework core functions
//

Database::Database(double initialError, double initialErrorDisparity, double initialRadius, unsigned int initialGroup, unsigned int initialMatchRange, double initialDenseDisparity){

    // Assign default parameters
    configError=initialError;
    configErrorDisparity=initialErrorDisparity;
    configRadius=initialRadius;
    configGroup=initialGroup;
    configMatchRange=initialMatchRange;
    configDenseDisparity=initialDenseDisparity;

    // Check consistency
    if(configGroup<3){
        std::cerr << "Warning : group value below 3" << std::endl;
    }

}

bool Database::getBootstrap(){
    
    // Check bootstrap condition
    if(viewpoints.size()<configGroup){
        return true;
    }else{
        return false;
    }

}

unsigned int Database::getGroup(){

    // Return group value
    return configGroup;

}

bool Database::getError(int pipeState, int loopMajor, int loopMinor){

    // Memory of error on transformation
    static double pushtError(-1.);

    // Error on transformation
    double tError(0.);
    
    // Iteration end condition flag
    bool returnValue(true);

    // Specific condition
    if(loopMinor>DB_LOOP_MAXITER){
        std::cerr << "Warning : maximum iterations count reached" << std::endl;
        returnValue=false;
    }

    // Specific condition
    if(pipeState==DB_MODE_MASS){
        returnValue=false;
    }

    // Detect maximum error on transformation
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        if(transforms[i]->getError()>tError){
            tError=transforms[i]->getError();
        }
    }

    // Apply error to deduce iterations stop condition
    if ( (std::fabs(tError-pushtError)<configError) ) {
        returnValue=false;
    }

    // Display information on iterations and errors
    std::cout << "step : " << std::setw(6) << loopMajor 
              << " | "
              << "iter : " << std::setw(6) << loopMinor
              << " | "
              << "state " << pipeState 
              << " | "
              << "error : " << tError
              << std::endl;

    // Pushing error
    pushtError=tError;

    // Send answser
    return returnValue;

}

void Database::getLocalViewpoints(Eigen::Vector3d position, std::vector<std::shared_ptr<Viewpoint>> *localViewpoints){

    // Detect amout of available last viewpoints
    int localCount = MIN(configMatchRange, viewpoints.size());

    // Add available to stack for matching
    for(auto availableViewpoint(viewpoints.end()-localCount); availableViewpoint != viewpoints.end(); ++availableViewpoint){
        localViewpoints->push_back(*availableViewpoint);
    }

}

void Database::addViewpoint(std::shared_ptr<Viewpoint> viewpoint){

    // Add new viewpoint to the stack
    viewpoints.push_back(viewpoint);

    // Add new transformation only if at least two viewpoints are in the stack
    if(viewpoint->getIndex() > 0){
        transforms.push_back(std::make_shared<Transform>());
    }

}

Structure * Database::addStructure(){

    // Create structure memory allocation
    auto newStructure = std::make_shared<Structure>(); 

    // Push new structure on the stack
    structures.push_back(newStructure); 

    // Return pointer to created structure
    return newStructure.get();

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
                structure = this->addStructure();
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

int Database::prepareState(int pipeState){

    // Check pipeline state
    if(pipeState==DB_MODE_LAST){

        // Set range : viewpoint
        rangeVlow  = viewpoints.size()-configGroup;
        rangeVhigh = viewpoints.size()-1;

        // Set range : transformation
        rangeTlow  = transforms.size()-configGroup+1;
        rangeThigh = transforms.size()-1;

        // Set range : structure
        rangeSlow  = 0;
        rangeShigh = structures.size()-1;

        // Set structure minimal activity
        stateStructure = STRUCTURE_PIONER;

    }else{

        // Set range : viewpoint
        rangeVlow  = 0;
        rangeVhigh = viewpoints.size()-1;

        // Set range : transformation
        rangeTlow  = 0;
        rangeThigh = transforms.size()-1;

        // Set range : structure
        rangeSlow  = 0;
        rangeShigh = structures.size()-1;

        // Set structure minimal activity
        stateStructure = STRUCTURE_NORMAL;

    }

    // Return state
    return pipeState;

}

void Database::prepareStructures(){

    // Parsing structures
    for(auto & structure: structures){

        // Ensure sorting of features based on their viewpoint index
        structure->sortFeatures();

        // Compute structure state
        structure->computeState(configGroup, rangeVhigh);

        // Check structure state
        if(structure->getState()==STRUCTURE_PIONER){

            // Reset structure
            structure->setReset();

        }

    }

}

void Database::prepareTransforms(){

    // Parsing transforms
    for(unsigned int i=rangeTlow; i<rangeThigh; i++){

        // Push transform scale
        transforms[i]->setScale();

    }

}

void Database::expungeStructures(){

    // Continuous index
    unsigned int index(0);

    // Parsing structure
    for(unsigned int i=0; i<structures.size(); i++){

        // Check structure state
        if(structures[i]->getState()>STRUCTURE_REMOVE){

            // Re-indexation
            if(index<i) structures[index]=structures[i];

            // Update continuous index
            index ++;

        }       

    }

    // Check re-indexation
    if(index<structures.size()){

        // Resize structures array
        structures.resize(index);

    }

}

void Database::broadcastScale(){

    // Scale factor
    double factor(0.);

    // Count variable
    int count(0);

    // Parsing transforms
    for(unsigned int i=rangeTlow; i<rangeThigh; i++){

        // Compute factor component
        factor+=transforms[i]->getScale();

        // Update counter
        count++;

    }

    // Compute scale factor
    factor/=double(count);

    // Parsing transforms
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){

        // Apply scale factor
        transforms[i]->setTranslationScale(factor);

    }

}

void Database::computeModels(int pipeState){
    
    // Check pipeline state
    if(pipeState==DB_MODE_MASS){

        // Avoid process
        return;

    }

    // Compute viewpoint relative feature position
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->computeModel();
        }
    }

}

void Database::computeCentroids(int pipeState){

    // Check pipeline state
    if(pipeState==DB_MODE_MASS){

        // Avoid process
        return;

    }

    // Reset centroids
    # pragma omp parallel for
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transforms[i]->resetCentroid();
    }
    
    // Distribute structure contribution to centroids
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            if(structures[i]->getHasScale(configGroup)){
                structures[i]->computeCentroid(transforms,rangeVlow);
            }
        }
    }

    // Compute centroids
    # pragma omp parallel for
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transforms[i]->computeCentroid();
    }

}

void Database::computeCorrelations(int pipeState){

    // Check pipeline state
    if(pipeState==DB_MODE_MASS){

        // Avoid process
        return;

    }

    // Reset correlation matrix
    # pragma omp parallel for
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transforms[i]->resetCorrelation();
    }

    // Distribute structure contribution to correlation matrix
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            if(structures[i]->getHasScale(configGroup)){
                structures[i]->computeCorrelation(transforms,rangeVlow);
            }
        }
    }

}

void Database::computePoses(int pipeState){

    // Check pipeline state
    if(pipeState==DB_MODE_MASS){

        // Avoid process
        return;

    }

    // Compute transformation rotation matrix and translation vector
    # pragma omp parallel for
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transforms[i]->computePose();
    }

}

void Database::computeNormalisePoses(int pipeState){

    // Count variable
    unsigned int count(0);

    // Check pipeline state
    if(pipeState==DB_MODE_MASS){

        // Avoid process
        return;

    }

    // Reset transformation norm mean
    transformMean=0.;

    // Accumulate transoformation norm
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transformMean+=transforms[i]->getTranslation()->norm();
        count++;
    }

    // Compute transformation norm mean
    transformMean/=double(count);

    // Transformation translation normalisation
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transforms[i]->setTranslationScale(transformMean);
    }

}

void Database::computeFrames(int pipeState){

    // Check pipeline state
    if(pipeState==DB_MODE_MASS){

        // Avoid process
        return;

    }

    // Assign initial orientation and position
    viewpoints[0]->resetFrame();

    // Compute absolute orientation and position
    for(unsigned int i=rangeTlow; i<=rangeThigh; i++){
        transforms[i]->computeFrame(viewpoints[i].get(),viewpoints[i+1].get());
    }

}

void Database::computeOriented(int pipeState){ /* param not needed */

    // Compute absolute orientation of features
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->computeOriented(rangeVlow);
        }
    }

}

void Database::computeOptimals(int pipeState){ /* param not needed */

    // Compute absolute optimal position of structures
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->computeOptimalPosition(rangeVlow);
        }
    }

}

void Database::computeRadii(int pipeState){ /* param not needed */

    // Compute feature radii according to optimal position
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->computeRadius(rangeVlow);
        }
    }

}

void Database::computeDisparityStatistics(int pipeState){ /* param not needed */

    // Count increment
    unsigned int countValue(0);

    // Reset values
    meanValue=0.;
    stdValue=0.;

    // Compute mean value and detect max value
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            countValue+=structures[i]->computeDisparityMean(&meanValue,rangeVlow);
        }
    }

    // Compute mean value
    meanValue/=double(countValue);

    // Compute standard deviation value
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->computeDisparityStd(&stdValue,meanValue,rangeVlow);
        }
    }

    // Compute standard deviation value
    stdValue=std::sqrt(stdValue/(countValue-1));

}

void Database::filterRadialRange(int pipeState){ /* param not needed */

    // Apply filtering condition
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->filterRadialRange(0.,configRadius,rangeVlow);
        }
    }

}

void Database::filterDisparity(int pipeState){

    // Filtering threshold value
    double thresholdValue(0.);

    // Check pipeline state
    if(pipeState==DB_MODE_MASS){
        thresholdValue=(2.*M_PI)*configDenseDisparity;
    }else{
        thresholdValue=stdValue*configErrorDisparity;
    }

    // Apply filtering condition
    # pragma omp parallel for schedule(dynamic)
    for(unsigned int i=rangeSlow; i<=rangeShigh; i++){
        if(structures[i]->getState()>=stateStructure){
            structures[i]->filterDisparity(thresholdValue,rangeVlow);
        }
    }


}

//
//  Framework exportation
//

void Database::exportStructure(std::string path, std::string mode, unsigned int major, unsigned int group){

    // Exportation variables
    std::fstream exportStream;
    std::stringstream filePath;
    std::stringstream fileCopy;
    cv::Vec3b color;

    // Create exportation path
    filePath << path << "/" << mode << "/" << std::setfill('0') << std::setw(4) << major << "_structure.xyz";

    // Create exportation stream
    exportStream.open(filePath.str(),std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create model exportation file" << std::endl;
        return;
    }

    // Structures exportation
    for(auto & structure: structures){

        // Export only structures with sufficiant viewpoints
        if(structure->getHasScale(group)){

            // Export structure position
            exportStream << (*structure->getPosition())(0) << " ";
            exportStream << (*structure->getPosition())(1) << " ";
            exportStream << (*structure->getPosition())(2) << " ";

            // Get structure mean color
            color=structure->getColor();

            // Export structure color - RGB888
            exportStream << std::to_string( color[2] ) << " ";
            exportStream << std::to_string( color[1] ) << " ";
            exportStream << std::to_string( color[0] ) << std::endl;

        }

    }

    // Delete exportation stream
    exportStream.close();

    // Copy structure file to main folder
    fileCopy << path << "/" << mode << "_structure.xyz";
    fs::copy(filePath.str(), fileCopy.str(),fs::copy_options::overwrite_existing);

}

void Database::exportPosition(std::string path, std::string mode, unsigned int major){

    // Exportation variables
    std::fstream exportStream;
    std::stringstream filePath;
    std::stringstream fileCopy;

    // Create exportation path
    filePath << path << "/" << mode << "/" << std::setfill('0') << std::setw(4) << major << "_position.xyz";

    // Create exportation stream
    exportStream.open(filePath.str(),std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create odometry exportation file" << std::endl;
        return;
    }

    // Viewpoints exportation
    for(auto & element: viewpoints){

        // Export viewpoint position with ad-hoc color
        exportStream << (*element->getPosition())(0) << " ";
        exportStream << (*element->getPosition())(1) << " ";
        exportStream << (*element->getPosition())(2) << " 255 0 255" << std::endl;

    }

    // Delete exportation stream
    exportStream.close();

    // Copy viewpoint file to main folder
    fileCopy << path << "/" << mode << "_position.xyz";
    fs::copy(filePath.str(), fileCopy.str(),fs::copy_options::overwrite_existing);

}

void Database::exportTransformation(std::string path, std::string mode, unsigned int major){

    // Exportation variables
    std::fstream exportStream;
    std::stringstream filePath;
    std::stringstream fileCopy;

    // Create exportation path
    filePath << path << "/" << mode << "/" << std::setfill('0') << std::setw(4) << major << "_transformation.dat";

    // Create exportation stream
    exportStream.open(filePath.str(),std::ios::out);
    if (exportStream.is_open() == false ){
        std::cerr << "unable to create transformation file" << std::endl;
    }

    // Export transformations
    for(auto & viewpoint: viewpoints){

        // Export viewpoint image UID
        exportStream << viewpoint->uid << " ";

        // Export viewpoint position - 3-vector
        exportStream << (*viewpoint->getPosition())(0) << " ";
        exportStream << (*viewpoint->getPosition())(1) << " ";
        exportStream << (*viewpoint->getPosition())(2) << " ";

        // Export viewpoint orientation - 3x3-matrix
        exportStream << (*viewpoint->getOrientation())(0,0) << " "; 
        exportStream << (*viewpoint->getOrientation())(0,1) << " ";
        exportStream << (*viewpoint->getOrientation())(0,2) << " ";
        exportStream << (*viewpoint->getOrientation())(1,0) << " "; 
        exportStream << (*viewpoint->getOrientation())(1,1) << " ";
        exportStream << (*viewpoint->getOrientation())(1,2) << " ";
        exportStream << (*viewpoint->getOrientation())(2,0) << " "; 
        exportStream << (*viewpoint->getOrientation())(2,1) << " ";
        exportStream << (*viewpoint->getOrientation())(2,2) << std::endl;

    }

    // Delete exportation stream
    exportStream.close();

    // Copy transformation file to main folder
    fileCopy << path << "/" << mode << "_transformation.dat";
    fs::copy(filePath.str(), fileCopy.str(),fs::copy_options::overwrite_existing);

}

void Database::exportConstraint(std::string path, std::string mode, unsigned int major, unsigned int group){

    // Exportation variables
    std::fstream exportStream;
    std::stringstream filePath;
    std::stringstream fileCopy;
    cv::Vec3b color;

    // Create exportation path
    filePath << path << "/" << mode << "/" << std::setfill('0') << std::setw(4) << major << "_constraint.dat";

    // Create exportation stream
    exportStream.open(filePath.str(),std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create odometry exportation file" << std::endl;
        return;
    }

    // Export constaints
    for(auto & structure: structures){

        // Export only structures with sufficiant viewpoints
        if(structure->getHasScale(group)){

            // Export structure position
            exportStream << (*structure->getPosition())(0) << " ";
            exportStream << (*structure->getPosition())(1) << " ";
            exportStream << (*structure->getPosition())(2) << " ";

            // Get structure mean color
            color=structure->getColor();

            // Export structure color - RGB888
            exportStream << std::to_string( color[2] ) << " ";
            exportStream << std::to_string( color[1] ) << " ";
            exportStream << std::to_string( color[0] ) << " ";

            // Export amount of viewpoint seen by the structure
            exportStream << structure->getFeatureCount() << " ";

            // Export index of viewpoints seen by the structure
            // These index corresponds, zero-based to the index in the viewpoint    
            // stack exported in the *_transformation.dat files
            for(unsigned int i(0); i<structure->getFeatureCount(); i++){

                // Export viewpoint index
                exportStream << structure->getFeatureViewpointIndex(i) << " ";

            }

            // Exportation line termination
            exportStream << std::endl;

        }

    }

    // Delete exportation stream
    exportStream.close();

    // Copy transformation file to main folder
    fileCopy << path << "/" << mode << "_constraint.dat";
    fs::copy(filePath.str(), fileCopy.str(),fs::copy_options::overwrite_existing);

}

//
//  Framework development features - Will be removed
//

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
        if(element->state==STRUCTURE_REMOVE)continue;
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
void Database::_exportStructureModel(std::string path, unsigned int major){
    if(viewpoints.size()<configGroup){
        return;
    }
    std::fstream stream;
    stream.open( path + "/debug/" + std::to_string(major) + "_strucutre" + ".uv3", std::ios::out | std::ios::binary );
    if(stream.is_open()==false){
        std::cerr << "unable to create structure model file" << std::endl;
    }

    double pos[3] = { 0. };
    unsigned char col[4] = { 2, 0, 0, 0 };
    char * posp= (char *)pos;
    char * colp= (char *)col;

    for(unsigned int i(0); i<structures.size(); i++){

        for(unsigned int j(0); j<structures[i]->features.size(); j++){

            Eigen::Vector3d dir((structures[i]->features[j]->viewpoint->position)-(structures[i]->position));

            pos[0]=structures[i]->position(0);
            pos[1]=structures[i]->position(1);
            pos[2]=structures[i]->position(2);
            col[1]=64+structures[i]->features.size()*4;
            col[2]=0;
            col[3]=0;
            stream.write(posp,3*sizeof(double));
            stream.write(colp,4*sizeof(unsigned char));

            pos[0]=structures[i]->position(0)+dir(0)*0.3;
            pos[1]=structures[i]->position(1)+dir(1)*0.3;
            pos[2]=structures[i]->position(2)+dir(2)*0.3;
            col[1]=64+structures[i]->features.size()*4;
            col[2]=0;
            col[3]=0;
            stream.write(posp,3*sizeof(double));
            stream.write(colp,4*sizeof(unsigned char));

            pos[0]=structures[i]->features[j]->viewpoint->position(0)-dir(0)*0.1;
            pos[1]=structures[i]->features[j]->viewpoint->position(1)-dir(1)*0.1;
            pos[2]=structures[i]->features[j]->viewpoint->position(2)-dir(2)*0.1;
            col[1]=0;
            col[2]=0;
            col[3]=64+structures[i]->features.size()*4;
            stream.write(posp,3*sizeof(double));
            stream.write(colp,4*sizeof(unsigned char));

            pos[0]=structures[i]->features[j]->viewpoint->position(0);
            pos[1]=structures[i]->features[j]->viewpoint->position(1);
            pos[2]=structures[i]->features[j]->viewpoint->position(2);
            col[1]=0;
            col[2]=0;
            col[3]=64+structures[i]->features.size()*4;
            stream.write(posp,3*sizeof(double));
            stream.write(colp,4*sizeof(unsigned char));

        }
    }
    stream.close();
}

