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

#include "framework-viewpointsource.hpp"

#include <iostream>
#include <experimental/filesystem>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>


namespace fs = std::experimental::filesystem;

ViewPointSourceFs::ViewPointSourceFs(std::vector<std::string> files){
	this->files = files;
	this->fileIndex = 0;
}

ViewPointSourceFs::ViewPointSourceFs(std::string folder){
	this->fileIndex = 0;

	// read the contents of the directory where the images are located
	fs::path pt = fs::u8path(folder);
	for (auto& p : fs::directory_iterator(pt)) {
		std::string str = p.path().u8string();
		if (str.substr(str.length()-3)=="bmp") {
			files.push_back(p.path().u8string());
		}
	}

	// sort the filenames alphabetically
	std::sort(files.begin(), files.end());
}

std::shared_ptr<Viewpoint> ViewPointSourceFs::next(){
	auto viewpoint = std::make_shared<Viewpoint>();
	std::string path = files[fileIndex];
	auto image = cv::imread(path, cv::IMREAD_COLOR);
	if (image.empty()) throw new std::runtime_error("imread path failure : " + path );
	viewpoint->setImage(image);
	fileIndex++;
	return viewpoint;
}

bool ViewPointSourceFs::hasNext(){
	return fileIndex != files.size();
}



