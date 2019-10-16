#include "framework-frontend.hpp"

#include "framework-stillcompute.hpp"
#include "framework-sparsefeature.hpp"

FrontendPicture::FrontendPicture(ViewPointSource * source, cv::Mat mask, ThreadPool *threadpool, Database *database) :
	source(source),
	threadpool(threadpool),
	mask(mask),
	featureExtractionQueue(2),
	database(database){
	featureExtractionThread = std::thread([this]{featureExtraction();});
}


void FrontendPicture::featureExtraction(){
	while(this->source->hasNext()){
		//Collect the next view point and add it into the database
		auto newViewpoint = source->next();
		auto m = threadpool->enqueue([this, newViewpoint] {
			akazeFeatures(newViewpoint->getImage(), &mask, newViewpoint->getCvFeatures(), newViewpoint->getCvDescriptor());
			return newViewpoint;
		});
		featureExtractionQueue.push(m);
	}
}


bool FrontendPicture::next() {
	auto newViewpoint = featureExtractionQueue.pop();

	//Check if the image is moving enough using features
	std::vector<cv::DMatch> lastViewpointMatches;
	if(lastViewpoint){
		gmsMatcher (
			newViewpoint->getCvFeatures(),
			newViewpoint->getCvDescriptor(),
			newViewpoint->getImage()->size(),
			lastViewpoint->getCvFeatures(),
			lastViewpoint->getCvDescriptor(),
			lastViewpoint->getImage()->size(),
			&lastViewpointMatches
		);
		double score = computeStillDistance(
			newViewpoint->getCvFeatures(),
			lastViewpoint->getCvFeatures(),
			&lastViewpointMatches,
			lastViewpoint->getImage()->size()
		);
//			std::cout << score << std::endl;
		if(score < 0.0005){
			return false;
		}
	}

	//profile("allocateFeaturesFromCvFeatures");
	newViewpoint->allocateFeaturesFromCvFeatures();

	//profile("misc");

	//Extrapolate the position of the newViewpoint
	//if(lastViewpoint){
	//	newViewpoint->setPosition(*lastViewpoint->getPosition());
	//} else {
	//	newViewpoint->setPosition(Eigen::Vector3d(0,0,0));
	//}
	database->extrapolateViewpoint(newViewpoint.get());

	//Get local viewpoints
	auto localViewpoints = std::vector<std::shared_ptr<Viewpoint>>();
	{
		auto viewpoints = database->getViewpoints();
		int localCount = MIN(3, viewpoints->size());
		for(auto i = viewpoints->end()-localCount;i != viewpoints->end(); ++i){
			localViewpoints.push_back(*i);
		}
	}
	uint32_t localViewpointsCount = localViewpoints.size();
	uint32_t newViewpointFeaturesCount = newViewpoint->getCvFeatures()->size();

	//Match local viewpoints to the new image
	//profile("gms + correlations");
	uint32_t *correlations = new uint32_t[newViewpointFeaturesCount*localViewpointsCount]; //-1 => empty
	memset(correlations, -1, newViewpointFeaturesCount*localViewpointsCount*sizeof(uint32_t));

//		#pragma omp parallel for
	for(uint32_t localViewpointIdx = 0; localViewpointIdx < localViewpointsCount; localViewpointIdx++){
		auto localViewpoint = localViewpoints[localViewpointIdx];
		if(localViewpoint == lastViewpoint){ //Reuse previously processed matches
			for(auto match : lastViewpointMatches){
				correlations[localViewpointIdx + match.queryIdx*localViewpointsCount] = match.trainIdx;
			}
		} else {
			std::vector<cv::DMatch> matches;
			gmsMatcher (
				newViewpoint->getCvFeatures(),
				newViewpoint->getCvDescriptor(),
				newViewpoint->getImage()->size(),
				localViewpoint->getCvFeatures(),
				localViewpoint->getCvDescriptor(),
				localViewpoint->getImage()->size(),
				&matches
			);

			for(auto match : matches){
				correlations[localViewpointIdx + match.queryIdx*localViewpointsCount] = match.trainIdx;
			}
		}
	}

	//Integrate the new image features into the structure
	//profile("aggregation");
	Structure** structures = new Structure*[localViewpointsCount];
	uint32_t* structuresOccurences = new uint32_t[localViewpointsCount];
	uint32_t structureNewCount = 0;
	uint32_t structureAggregationCount = 0;
	uint32_t structureFusionCount = 0;
	for(uint32_t queryIdx = 0;queryIdx < newViewpoint->getCvFeatures()->size(); queryIdx++){
		uint32_t *correlationsPtr = correlations + queryIdx*localViewpointsCount; //Used to iterate over the given lines

		uint32_t structuresCount = 0;
		uint32_t matchCount = 0;

		//Collect all the existing structures of the matches and count their occurences
		for(uint32_t localIdx = 0;localIdx < localViewpointsCount;localIdx++){
			uint32_t trainIdx = correlationsPtr[localIdx];
			if(trainIdx != 0xFFFFFFFF){
				matchCount++;
				auto localFeature = localViewpoints[localIdx]->getFeatureFromCvIndex(trainIdx);
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
				structure = database->newStructure();
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

		//Integrate all orphan feature into the common structure
		for(uint32_t localIdx = 0;localIdx < localViewpointsCount;localIdx++){
			uint32_t trainIdx = correlationsPtr[localIdx];
			if(trainIdx != 0xFFFFFFFF){
				auto localFeature = localViewpoints[localIdx]->getFeatureFromCvIndex(trainIdx);
				if(!localFeature->structure){
					structure->addFeature(localViewpoints[localIdx]->getFeatureFromCvIndex(trainIdx));
				}
			}
		}

		auto newFeature = newViewpoint->getFeatureFromCvIndex(queryIdx);
		assert(!newFeature->structure);
		structure->addFeature(newFeature);
	}
	delete[] correlations;
	delete[] structures;
	delete[] structuresOccurences;
	std::cout << "structureNewCount=" << structureNewCount << " structureAggregationCount=" << structureAggregationCount << " structureFusionCount=" << structureFusionCount << std::endl;

	lastViewpoint = newViewpoint;

	//profile("display");
	database->addViewpoint(newViewpoint);
	database->extrapolateStructure();
	database->displayViewpointStructures(newViewpoint.get());
	cv::waitKey(100); //Wait 100 ms give opencv the time to display the GUI

	//As currently we aren't using the image, we can just throw it aways to avoid memory overflow.
	newViewpoint->getImage()->deallocate(); //TODO
	return true;
}
