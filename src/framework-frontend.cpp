#include "framework-frontend.hpp"

#include "framework-stillcompute.hpp"
#include "framework-sparsefeature.hpp"
#include "framework-utiles.hpp"

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
//		if(score < 0.0005){
        if(score < 0.002){
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

    Eigen::Matrix3d randRot(
        Eigen::AngleAxisd((2.0*rand()*M_PI)/RAND_MAX, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd((2.0*rand()*M_PI)/RAND_MAX, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd((2.0*rand()*M_PI)/RAND_MAX, Eigen::Vector3d::UnitZ())
    );
	for(uint32_t mid = 0;mid < model.size();mid++){
		auto m = model[mid];
		auto position = (m-o);
		if((m-o).norm() < distanceMax){
			auto f = new Feature();
			auto noiseFactor = baseNoise + (1.0*rand()/RAND_MAX < badMatchRate ? badMatchNoise : 0);
			auto noise = Eigen::Vector3d(distanceMax*noiseFactor*rand()/RAND_MAX,distanceMax*noiseFactor*rand()/RAND_MAX,distanceMax*noiseFactor*rand()/RAND_MAX);
	        f->setDirection((randRot*(position + noise)).normalized());
	        f->setRadius(1., 0.);
	        f->setViewpointPtr(newViewpoint.get());
	        f->setStructurePtr(NULL);
	        f->inliner = mid;
			newViewpoint->addFeature(f);
		}
	}

    newViewpoint->resetFrame();

	std::vector<std::shared_ptr<Viewpoint>> localViewpoints;
	database->getLocalViewpoints(newViewpoint->position, &localViewpoints);

	uint32_t localViewpointsCount = localViewpoints.size();
	uint32_t newViewpointFeaturesCount = newViewpoint->features.size();

	//Match local viewpoints to the new image
	uint32_t *correlations = new uint32_t[newViewpointFeaturesCount*localViewpointsCount]; //-1 => empty
	memset(correlations, -1, newViewpointFeaturesCount*localViewpointsCount*sizeof(uint32_t));

    // loop on local viewpoint
	for(uint32_t localViewpointIdx = 0; localViewpointIdx < localViewpointsCount; localViewpointIdx++){

        // extract current local viewpoint
		auto localViewpoint = localViewpoints[localViewpointIdx];

        // loop on current local viewpoint features
		for(uint32_t localFeatureIndex = 0;localFeatureIndex < localViewpoint->features.size();localFeatureIndex++){

            // extract "current current" local viewpoint feature
			auto localFeatureInliner = localViewpoint->features[localFeatureIndex]->inliner;

            // loop on new viewpoint features
			for(uint32_t newFeatureIndex = 0;newFeatureIndex < newViewpoint->features.size();newFeatureIndex++){

                // extract new viewpoint current feature
				auto newFeatureInliner = newViewpoint->features[newFeatureIndex]->inliner;

                // compare feature point index in the source point cloud (synthetic model)
				if(localFeatureInliner == newFeatureInliner){

                    // update the feature correlation matrix
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

    exitRetain();
}


#include "../lib/libflow/src/Cache.h"
bool FrontendDense::next() {
    if(!source->hasNext()) return false;
    auto newViewpoint = source->next();
    const int margin = 4;

    if(database->viewpoints.size() != 0){
        auto lastViewpoint = database->viewpoints.back();
        cv::Mat u,v;

        auto imageLast = cv::Mat();
        lastViewpoint->image.convertTo( imageLast, CV_64FC3 );
        imageLast /= 255.0;

        auto imageNew = cv::Mat();
        newViewpoint->image.convertTo( imageNew, CV_64FC3 );
        imageNew /= 255.0;

        ofCache(imageLast, imageNew, u, v, ofCacheFolder); //TODO waning, last viewpoint may not have image
        cv::Mat stencil = cv::Mat::zeros(lastViewpoint->image.rows, lastViewpoint->image.cols, CV_8UC1);

        //Extend existing structures with lastViewpoint matches
        for(auto lastFeature: lastViewpoint->features) if(lastFeature->structure) {
            //TODO use bilinear_sample
            auto newPosition = lastFeature->position + Eigen::Vector2f(
                u.at<float>(lastFeature->position.y(), lastFeature->position.x()),
                v.at<float>(lastFeature->position.y(), lastFeature->position.x())
            );
//            auto newPosition = lastFeature->position + Eigen::Vector2f(
//                bilinear_sample((double*)u.data, lastFeature->position.x(), lastFeature->position.y(), lastViewpoint->image.cols),
//                bilinear_sample((double*)v.data, lastFeature->position.x(), lastFeature->position.y(), lastViewpoint->image.cols)
//            );
            if(newPosition.x() < margin || newPosition.y() < margin || newPosition.x() >= newViewpoint->image.cols -margin || newPosition.y() >= newViewpoint->image.rows -margin) continue;
            if(!mask.at<uint8_t>(newPosition.y(), newPosition.x())) continue;

            auto newFeature = new Feature();
            newFeature->setFeature(newPosition.x(), newPosition.y(), newViewpoint->image.cols, newViewpoint->image.rows);
            newFeature->setViewpointPtr(newViewpoint.get());
            newFeature->setColor(newViewpoint->image.empty() ? cv::Vec3b(255,255,255) : newViewpoint->image.at<cv::Vec3b>(newPosition.y(), newPosition.x()));
            newViewpoint->addFeature(newFeature);
            lastFeature->structure->addFeature(newFeature);

            stencil.at<uint8_t>(lastFeature->position.y(), lastFeature->position.x()) = 255;
//            cv::circle(stencil, cv::Point(lastFeature->position.x(), lastFeature->position.y()), 2, cv::Scalar(255,255,255),1,cv::FILLED,0);
        }

//        cv::namedWindow( "stencil", cv::WINDOW_KEEPRATIO );
//        imshow( "stencil", stencil);
//        cv::waitKey(0);

        //Create new structure for empty area
        for(int y = margin;y < stencil.rows-margin;y++){
            for(int x = margin;x < stencil.cols-margin;x++){
                if(!stencil.at<uint8_t>(y, x)){
                    auto newPosition = Eigen::Vector2f(
                        x + u.at<float>(y, x),
                        y + v.at<float>(y, x)
                    );

                    if(newPosition.x() < margin || newPosition.y() < margin || newPosition.x() >= newViewpoint->image.cols -margin || newPosition.y() >= newViewpoint->image.rows -margin) continue;
                    if(!mask.at<uint8_t>(y, x)) continue;
                    if(!mask.at<uint8_t>(newPosition.y(), newPosition.x())) continue;

                    auto newStructure = database->newStructure(newViewpoint.get());

                    auto lastFeature = new Feature();
                    lastFeature->setFeature(x, y, lastViewpoint->image.cols, lastViewpoint->image.rows);
                    lastFeature->setViewpointPtr(lastViewpoint.get());
                    lastFeature->setColor(lastViewpoint->image.empty() ? cv::Vec3b(255,255,255) : lastViewpoint->image.at<cv::Vec3b>(y, x));
                    lastViewpoint->addFeature(lastFeature);
                    newStructure->addFeature(lastFeature);

                    auto newFeature = new Feature();
                    newFeature->setFeature(newPosition.x(), newPosition.y(), newViewpoint->image.cols, newViewpoint->image.rows);
                    newFeature->setViewpointPtr(newViewpoint.get());
                    newFeature->setColor(newViewpoint->image.empty() ? cv::Vec3b(255,255,255) : newViewpoint->image.at<cv::Vec3b>(newPosition.y(), newPosition.x()));
                    newViewpoint->addFeature(newFeature);
                    newStructure->addFeature(newFeature);
                }
            }
        }
    }

    newViewpoint->setIndex(database->viewpoints.size());
    database->addViewpoint(newViewpoint);
    exitRetain();
    if(!source->hasNext()) exitRelease();
    return true;
}
