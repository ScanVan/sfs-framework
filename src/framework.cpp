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

int main(){
	std::cout << "Hello world!" << std::endl;
	auto source = ViewPointSourceFs("/media/dolu/SCANVAN10TB/record/camera_40008603-40009302/20190319-103441_SionCar1");
	auto mask = cv::imread("/home/dolu/pro/scanvan/fs", cv::IMREAD_GRAYSCALE);
	auto database = Database();

	std::shared_ptr<Viewpoint> lastViewpoint;
	while(source.hasNext()){
		//Collect the next view point and add it into the database
		auto newViewpoint = source.next();
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

//		std::cout << "X" << std::endl;
		cv::namedWindow("miaou", cv::WINDOW_NORMAL);
		cv::imshow("miaou", *newViewpoint->getImage());
		cv::waitKey(0);

		lastViewpoint = newViewpoint;
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

            loopError = 0.0; // getError()
            if ( fabs( loopError - pushError ) < database.getParameter()->getError() ) {
                loopFlag == false;
            }

            loopIteration ++;
            std::cout << "algorithm iteration : " << loopIteration << std::endl;

        }

        // algorithm optimisation loop

	}
	return 0;
}
