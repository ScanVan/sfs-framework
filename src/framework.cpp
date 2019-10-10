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

#include "framework.hpp"

#include "framework-stillcompute.hpp"
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include "framework-utiles.hpp"
#include "ThreadPool.h"

int main(int argc, char *argv[]){
	//profile("boot");
	assert(argc == 2);
	std::cout << "Hello world!" << std::endl;

	YAML::Node config = YAML::LoadFile(argv[1]);

	ViewPointSource *source = NULL;
	auto sourceType = config["source"]["type"].as<std::string>();
	if(sourceType == "FOLDER") source = new ViewPointSourceFs(config["source"]["path"].as<std::string>());
	auto mask = cv::imread(config["source"]["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
	auto database = Database();

    ThreadPool threadpool(8);


    BlockingQueue<std::shared_ptr<Viewpoint>> featureExtractionQueue(2);
    auto featureExtractionThread = std::thread([&]{
		while(source->hasNext()){
			//Collect the next view point and add it into the database
			auto newViewpoint = source->next();
			auto m = threadpool.enqueue([&mask, newViewpoint] {
//				std::cout << "AKAZE " << newViewpoint << std::endl;
				akazeFeatures(newViewpoint->getImage(), &mask, newViewpoint->getCvFeatures(), newViewpoint->getCvDescriptor());
//				std::cout << "AKAZE DONE " << newViewpoint << std::endl;
				return newViewpoint;
			});
			featureExtractionQueue.push(m);
		}
    });


    int loopMajor(1);


	std::shared_ptr<Viewpoint> lastViewpoint;
	while(true){
		auto newViewpoint = featureExtractionQueue.pop();

//		std::cout << "POP" << std::endl;

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
				continue; //Drop the image
			}
		}

		//profile("allocateFeaturesFromCvFeatures");
		newViewpoint->allocateFeaturesFromCvFeatures();

		//profile("misc");

		//Extrapolate the position of the newViewpoint
		if(lastViewpoint){
			newViewpoint->setPosition(*lastViewpoint->getPosition());
		} else {
			newViewpoint->setPosition(Eigen::Vector3d(0,0,0));
		}

		//Get local viewpoints
		auto localViewpoints = std::vector<std::shared_ptr<Viewpoint>>();
		{
			auto viewpoints = database.getViewpoints();
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
					structure = database.newStructure();
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
			structure->addFeature(newViewpoint->getFeatureFromCvIndex(queryIdx));
		}
		delete[] correlations;
		delete[] structures;
		delete[] structuresOccurences;
		std::cout << "structureNewCount=" << structureNewCount << " structureAggregationCount=" << structureAggregationCount << " structureFusionCount=" << structureFusionCount << std::endl;

		lastViewpoint = newViewpoint;

		//profile("display");
		database.addViewpoint(newViewpoint);
		database.displayViewpointStructures(newViewpoint.get());
		cv::waitKey(100); //Wait 100 ms give opencv the time to display the GUI

		//As currently we aren't using the image, we can just throw it aways to avoid memory overflow.
		newViewpoint->getImage()->deallocate(); //TODO

//		cv::namedWindow("miaou", cv::WINDOW_NORMAL);
//		cv::imshow("miaou", *newViewpoint->getImage());
//		cv::waitKey(0);

        //
        // geometry estimation solver - begin
        //

        // check for at least two pushed viewpoint
        if ( database.getViewpointCount() < 2 ) {

            // avoid optimisation
            continue;

        }

        double loopError( 1.0 );
        double pushError( 0.0 );
        double paramError( config["algorithm"]["error"].as<double>() );
        double paramDisparity( config["algorithm"]["disparity"].as<double>() );
        double paramRadius( config["algorithm"]["radius"].as<double>() );

        bool loopFlag( true );

        int loopIteration( 0 );

        // debug
        //database._exportMatch( config["source"]["pathTest"].as<std::string>() );
        // debug

        while ( loopFlag == true ) {

            database.computeModels();
            database.computeCentroids();
            database.computeCorrelations();
            database.computePoses();
            database.computeFrame();
            database.computeOptimal();
            database.computeRadius();
            database.computeStatistics();
            database.computeFilter(paramDisparity,paramRadius);

            // debug
            std::cerr << "debug : exporting iteration state ..." << std::endl;
            database._exportState( config["source"]["pathTest"].as<std::string>(), loopMajor, loopIteration );
            // debug

            loopError = database.computeError();
            if ( fabs( loopError - pushError ) < paramError ) {
                loopFlag = false;
            } else {
                pushError=loopError;
            }

            loopIteration ++;
            std::cout << "step : " << loopMajor << " | iteration : " << loopIteration << " | error : " << loopError << std::endl;

        }

        //database._exportInitialPair( config["source"]["pathTest"].as<std::string>() ); return(1);

        loopMajor ++;

        // debug
        if (loopMajor>2) return(1);
        // debug

	}

    database.exportModel(config["export"]["path"].as<std::string>());
    database.exportOdometry(config["export"]["path"].as<std::string>());

	return 0;
}

