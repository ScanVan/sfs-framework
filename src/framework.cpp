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

int main(){
	std::cout << "Hello world!" << std::endl;
	auto source = ViewPointSourceFs("/media/dolu/SCANVAN10TB/record/camera_40008603-40009302/20190319-103441_SionCar1");
	auto mask = cv::imread("/home/dolu/pro/scanvan/fs", cv::IMREAD_GRAYSCALE);
	auto database = Database();

	uint32_t index = 0;
	std::shared_ptr<Viewpoint> lastViewpoint;
	while(source.hasNext()){
		//Collect the next view point and add it into the database
		auto newViewpoint = source.next();
		if((index++) % 8 != 0) continue; //For debug purposes
		database.addViewpoint(newViewpoint);

		//Process the viewpoint sparse features
		akazeFeatures(newViewpoint->getImage(), &mask, newViewpoint->getFeatures(), newViewpoint->getDescriptor());

		//Check if the image is moving enough using features


		//Extrapolate the position of the newViewpoint

		std::cout << "X" << std::endl;
//		cv::namedWindow("miaou", cv::WINDOW_NORMAL);
//		cv::imshow("miaou", *viewpoint->getImage());
//		cv::waitKey(0);

		lastViewpoint = newViewpoint;
	}
	return 0;
}
