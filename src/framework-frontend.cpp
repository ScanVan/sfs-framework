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
    exitRetain();
	while(this->source->hasNext()){
		//Collect the next view point and add it into the database
		auto newViewpoint = source->next();
		auto m = threadpool->enqueue([this, newViewpoint] {
			akazeFeatures(newViewpoint->getImage(), &mask, newViewpoint->getCvFeatures(), newViewpoint->getCvDescriptor());
			return newViewpoint;
		});
	    exitRetain();
		featureExtractionQueue.push(m);
	}
	exitRelease();
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
		    exitRelease();
			return false;
		}
	}

	newViewpoint->allocateFeaturesFromCvFeatures();

	//Extrapolate the position of the newViewpoint
    newViewpoint->resetFrame();

	//Get local viewpoints
	std::vector<std::shared_ptr<Viewpoint>> localViewpoints;
	database->getLocalViewpoints(newViewpoint->position, &localViewpoints);

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
	database->aggregate(&localViewpoints, newViewpoint.get(), correlations);
	delete[] correlations;

	lastViewpoint = newViewpoint;

	database->addViewpoint(newViewpoint);
	return true;
}



FrontendCloudpoint::FrontendCloudpoint(
        Database *database,
        std::string modelPath,
        std::string odometryPath,
        double distanceMax,
        double badMatchRate,
        double baseNoise, double badMatchNoise) : database(database), distanceMax(distanceMax), badMatchRate(badMatchRate), baseNoise(baseNoise), badMatchNoise(badMatchNoise){
	std::ifstream m(modelPath);
	while(true){
		Eigen::Vector3d p;
		if(!(m >> p[0] >> p[1] >> p[2])) break;
		model.push_back(p);
	}

	std::ifstream o(odometryPath);
	while(true){
		Eigen::Vector3d p;
		if(!(o >> p[0] >> p[1] >> p[2])) break;
		odometry.push_back(p);
	}
	exitRetain();
}

bool FrontendCloudpoint::next(){
	if(viewpointIndex == odometry.size()) return false;
	exitRetain();
	auto newViewpoint = std::make_shared<Viewpoint>();

	//Extract feature from nearby model points
	auto o = odometry[viewpointIndex++];
    if(viewpointIndex == odometry.size()) exitRelease();

//	auto origin = odometry[0];
    Eigen::Matrix3d randRot(
        Eigen::AngleAxisd((2.0*rand()*M_PI)/RAND_MAX, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd((2.0*rand()*M_PI)/RAND_MAX, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd((2.0*rand()*M_PI)/RAND_MAX, Eigen::Vector3d::UnitZ())
    );
	for(uint32_t mid = 0;mid < model.size();mid++){
		auto m = model[mid];
		auto position = (m-o);
		if((m-o).norm() < distanceMax){
			Feature f;
			auto noiseFactor = baseNoise + (1.0*rand()/RAND_MAX < badMatchRate ? badMatchNoise : 0);
			auto noise = Eigen::Vector3d(distanceMax*noiseFactor*rand()/RAND_MAX,distanceMax*noiseFactor*rand()/RAND_MAX,distanceMax*noiseFactor*rand()/RAND_MAX);
	        f.setDirection((randRot*(position + noise)).normalized());
//	        f.setRadius(position.norm()+0.1, 0.);
	        f.setRadius(1., 0.);
	        f.setViewpointPtr(newViewpoint.get());
	        f.setStructurePtr(NULL);
            //f.setState(false);
	        f.inliner = mid;
			newViewpoint->addFeature(f);
		}
	}

//	newViewpoint->setPose(Eigen::Matrix3d::Identity(), o-origin);
    newViewpoint->resetFrame();

	std::vector<std::shared_ptr<Viewpoint>> localViewpoints;
	database->getLocalViewpoints(newViewpoint->position, &localViewpoints);

	uint32_t localViewpointsCount = localViewpoints.size();
	uint32_t newViewpointFeaturesCount = newViewpoint->features.size();

	//Match local viewpoints to the new image
	uint32_t *correlations = new uint32_t[newViewpointFeaturesCount*localViewpointsCount]; //-1 => empty
	memset(correlations, -1, newViewpointFeaturesCount*localViewpointsCount*sizeof(uint32_t));
	for(uint32_t localViewpointIdx = 0; localViewpointIdx < localViewpointsCount; localViewpointIdx++){
		auto localViewpoint = localViewpoints[localViewpointIdx];
		for(uint32_t localFeatureIndex = 0;localFeatureIndex < localViewpoint->features.size();localFeatureIndex++){
			auto localFeatureInliner = localViewpoint->features[localFeatureIndex].inliner;
			for(uint32_t newFeatureIndex = 0;newFeatureIndex < newViewpoint->features.size();newFeatureIndex++){
				auto newFeatureInliner = newViewpoint->features[newFeatureIndex].inliner;
				if(localFeatureInliner == newFeatureInliner){
					correlations[localViewpointIdx + newFeatureIndex*localViewpointsCount] = localFeatureIndex;
				}
			}
		}
	}

	database->aggregate(&localViewpoints, newViewpoint.get(), correlations);
	delete[] correlations;

	database->addViewpoint(newViewpoint);
	return true;
}





FrontendDense::FrontendDense(ViewPointSource * source, cv::Mat mask,Database *database, std::string ofCacheFolder) :
    source(source),
    mask(mask),
    database(database),
    ofCacheFolder(ofCacheFolder){

}


#include "../lib/libflow/src/Cache.h"
bool FrontendDense::next() {
    if(!source->hasNext()) return false;
    auto newViewpoint = source->next();
    const int margin = 4;

    if(database->viewpoints.size() != 0){
        auto lastViewpoint = database->viewpoints.back();
        cv::Mat u,v;
        ofCache(lastViewpoint->image, newViewpoint->image, u, v, ofCacheFolder); //TODO waning, last viewpoint may not have image
        cv::Mat stencil = cv::Mat::zeros(lastViewpoint->image.rows, lastViewpoint->image.cols, CV_8UC1);

        //Extend existing structures with lastViewpoint matches
        for(auto &lastFeature: lastViewpoint->features) if(lastFeature.structure) {
            //TODO use bilinear_sample
            auto newPosition = lastFeature.position + Eigen::Vector2f(
                u.at<double>(lastFeature.position.y(), lastFeature.position.x()),
                v.at<double>(lastFeature.position.y(), lastFeature.position.x())
            );
            if(newPosition.x() < margin || newPosition.y() < margin || newPosition.x() >= newViewpoint->image.cols -margin || newPosition.y() >= newViewpoint->image.rows -margin) continue;
            if(!mask.at<uint8_t>(newPosition.y(), newPosition.x())) continue;

            Feature newFeature;
            newFeature.setFeature(newPosition.x(), newPosition.y(), newViewpoint->image.cols, newViewpoint->image.rows);
            newFeature.setViewpointPtr(newViewpoint.get());
            auto color = newViewpoint->image.empty() ? cv::Vec3b(255,255,255) : newViewpoint->image.at<cv::Vec3b>(newPosition.y(), newPosition.x());
            newFeature.setColor(color);
            lastFeature.structure->addFeature(newViewpoint->addFeature(newFeature));

            cv::circle(stencil, cv::Point(lastFeature.position.x(), lastFeature.position.y()), 2, cv::Scalar(255,255,255),1,cv::FILLED,0);
        }

        //Create new structure for empty area
        for(int y = margin;y < stencil.rows-margin;y++){
            for(int x = margin;x < stencil.cols-margin;x++){
                if(!stencil.at<uint8_t>(y,x)){
                    auto newPosition = Eigen::Vector2f(
                        x + u.at<double>(y, x),
                        y + v.at<double>(y, x)
                    );

                    if(newPosition.x() < margin || newPosition.y() < margin || newPosition.x() >= newViewpoint->image.cols -margin || newPosition.y() >= newViewpoint->image.rows -margin) continue;
                    if(!mask.at<uint8_t>(y, x)) continue;
                    if(!mask.at<uint8_t>(newPosition.y(), newPosition.x())) continue;

                    auto newStructure = database->newStructure(lastViewpoint.get());

                    Feature lastFeature;
                    lastFeature.setFeature(x, y, lastViewpoint->image.cols, lastViewpoint->image.rows);
                    lastFeature.setViewpointPtr(lastViewpoint.get());
                    lastFeature.setColor(lastViewpoint->image.empty() ? cv::Vec3b(255,255,255) : lastViewpoint->image.at<cv::Vec3b>(y, x));
                    newStructure->addFeature(newViewpoint->addFeature(lastFeature));

                    Feature newFeature;
                    newFeature.setFeature(newPosition.y(), newPosition.x(), newViewpoint->image.cols, newViewpoint->image.rows);
                    newFeature.setViewpointPtr(newViewpoint.get());
                    newFeature.setColor(newViewpoint->image.empty() ? cv::Vec3b(255,255,255) : newViewpoint->image.at<cv::Vec3b>(newPosition.y(), newPosition.x()));
                    newStructure->addFeature(newViewpoint->addFeature(newFeature));
                }
            }
        }
    }

    database->addViewpoint(newViewpoint);
    return true;
}
