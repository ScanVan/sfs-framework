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
int main(int argc, char *argv[]){
	assert(argc == 2);
	std::cout << "Hello world!" << std::endl;

	YAML::Node config = YAML::LoadFile(argv[1]);

	ViewPointSource *source = NULL;
	auto sourceType = config["source"]["type"].as<std::string>();
	if(sourceType == "FOLDER") source = new ViewPointSourceFs(config["source"]["path"].as<std::string>());
	auto mask = cv::imread(config["source"]["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
	auto database = Database();

    // temporary path specification
    database.setPath( std::string("/some/record/path"), std::string("/some/model/path"));

	std::shared_ptr<Viewpoint> lastViewpoint;
	while(source->hasNext()){
		//Collect the next view point and add it into the database
		auto newViewpoint = source->next();
//		if((index++) % 8 != 0) continue; //For debug purposes
		database.addViewpoint(newViewpoint);

		//Process the viewpoint sparse features
		akazeFeatures(newViewpoint->getImage(), &mask, newViewpoint->getFeatures(), newViewpoint->getDescriptor());

		//Check if the image is moving enough using features
		if(lastViewpoint){
			std::vector<cv::DMatch> matches;
			gmsMatcher (
				lastViewpoint->getFeatures(),
				lastViewpoint->getDescriptor(),
				lastViewpoint->getImage()->size(),
				newViewpoint->getFeatures(),
				newViewpoint->getDescriptor(),
				newViewpoint->getImage()->size(),
				&matches
			);
			double score = computeStillDistance(
				lastViewpoint->getFeatures(),
				newViewpoint->getFeatures(),
				&matches,
				lastViewpoint->getImage()->size()
			);
			std::cout << score << std::endl;
			if(score < 0.0005){
				continue; //Drop the image
			}
		}

		//Extrapolate the position of the newViewpoint
		if(lastViewpoint){
			newViewpoint->setPosition(*lastViewpoint->getPosition());
		} else {
			newViewpoint->setPosition(Eigen::Vector3d(0,0,0));
		}
		lastViewpoint = newViewpoint;

		continue;

		//Get local viewpoints
		auto localViewpoints = std::vector<std::shared_ptr<Viewpoint>>();
		{
			auto viewpoints = database.getViewpoints();
			int localCount = MIN(3, viewpoints->size());
			for(auto i = viewpoints->end()-localCount;i != viewpoints->end(); ++i){
				localViewpoints.push_back(*i);
			}
		}

		//Match local viewpoints to the new image
		std::vector<std::vector<cv::DMatch>> matches;
		for(uint32_t localViewpointIdx = 0; localViewpointIdx < localViewpoints.size(); localViewpointIdx++){
			auto localViewpoint = localViewpoints[localViewpointIdx];
			gmsMatcher (
				newViewpoint->getFeatures(),
				newViewpoint->getDescriptor(),
				newViewpoint->getImage()->size(),
				localViewpoint->getFeatures(),
				localViewpoint->getDescriptor(),
				localViewpoint->getImage()->size(),
				&(matches[localViewpointIdx])
			);
		}

		//Integrate the new image features into the structure
//		for(uint32_t localFeatureIdx = 0; localFeatureIdx < )
		std::cout << "miaou" << std::endl;

//		cv::namedWindow("miaou", cv::WINDOW_NORMAL);
//		cv::imshow("miaou", *newViewpoint->getImage());
//		cv::waitKey(0);


//		continue;
        // algorithm optimisation loop

        bool loopFlag( true );

        double loopError( 1.0 );
        double pushError( 0.0 );

        int loopIteration( 0 );

        while ( loopFlag == true ) {

            database.computeModels();
            database.computeCorrelations();
            database.computePoses();
            database.computeFrame();
            database.computeRadius();

            loopError = database.computeError();
            if ( fabs( loopError - pushError ) < database.getParameter()->getError() ) {
                loopFlag == false;
            } else {
                pushError=loopError;
            }

            loopIteration ++;
            std::cout << "algorithm iteration : " << loopIteration << std::endl;

        }

        // algorithm optimisation loop

	}

    // exportation
    database.exportModel();
    database.exportOdometry();

	return 0;
}
