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
#include <ctime>
#include <fstream>

namespace fs = std::experimental::filesystem;

ViewPointSourceFs::ViewPointSourceFs(std::vector<std::string> files){
	this->files = files;
	this->fileIndex = 0;
}

ViewPointSourceFs::ViewPointSourceFs(std::string folder, double scale, std::string firstFile, std::string lastFile, uint32_t increment) : scale(scale), increment(increment) {
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

	firstFile = firstFile == "" ? files.front() : folder + "/" + firstFile;
	lastFile = lastFile == "" ? files.back() : folder + "/" + lastFile;

    fileIndex = findInVector(files, firstFile).second;
    fileLastIndex = findInVector(files, lastFile).second;

    if(fileIndex < 0) throw std::runtime_error("can't fine first file");
    if(fileLastIndex < 0) throw std::runtime_error("can't fine last file");
}

std::shared_ptr<Viewpoint> ViewPointSourceFs::next(){
	auto viewpoint = std::make_shared<Viewpoint>();
	std::string path = files[fileIndex];
    auto read = cv::Mat();
	auto image = cv::Mat();
    read = cv::imread(path, cv::IMREAD_COLOR);
	cv::resize(read, image, cv::Size(), scale, scale, cv::INTER_AREA );
    read.release();

	if (image.empty()) throw new std::runtime_error("imread path failure : " + path );
	viewpoint->setImage(image);
	viewpoint->setImageDimension(image.cols, image.rows);
    image.release();

	struct tm tm;
	auto fileName = fs::path(path).filename();
	auto dateString = fileName.string().substr(0, fileName.string().size() - fileName.extension().string().size());
	viewpoint->microsecond = std::stoi(dateString.substr(16, 16+6));
	dateString = dateString.substr(0, 15);
	dateString.insert(13,"-");
	dateString.insert(11,"-");
	dateString.insert(6,"-");
	dateString.insert(4,"-");

	assert(strptime(dateString.c_str(), "%Y-%m-%d-%H-%M-%S", &tm));
	viewpoint->time = mktime(&tm);

	viewpoint->uid = fileName.string();

	//fileIndex++;
    fileIndex += increment;
	return viewpoint;
}

bool ViewPointSourceFs::hasNext(){
	return fileIndex <= fileLastIndex;
}








ViewPointSourceWithOdometry::ViewPointSourceWithOdometry(std::string viewpointsPath, std::string pictureFolder, double scale, std::string firstFile, std::string lastFile) : scale(scale), pictureFolder(pictureFolder){
    std::ifstream listFile(viewpointsPath);

    ViewPointInfo info;

    fileIndex = -1;
    fileLastIndex = -1;

    while (listFile >>
            info.fileName >>
            info.position[0] >> info.position[1] >> info.position[2] >>
            info.orientation(0,0) >> info.orientation(0,1) >> info.orientation(0,2) >>
            info.orientation(1,0) >> info.orientation(1,1) >> info.orientation(1,2) >>
            info.orientation(2,0) >> info.orientation(2,1) >> info.orientation(2,2))
    {
        if(info.fileName == firstFile) fileIndex = list.size();
        if(info.fileName == lastFile) fileLastIndex = list.size();
        list.push_back(info);
    }

    if(firstFile == "") fileIndex = 0;
    if(lastFile == "") fileLastIndex = list.size()-1;

    if(fileIndex < 0) throw std::runtime_error("can't fine first file");
    if(fileLastIndex < 0) throw std::runtime_error("can't fine last file");
}

std::shared_ptr<Viewpoint> ViewPointSourceWithOdometry::next(){
    auto viewpoint = std::make_shared<Viewpoint>();
    auto info = list[fileIndex];

    auto image = cv::Mat();
    auto path = pictureFolder + "/" + info.fileName;
    cv::resize(cv::imread(path, cv::IMREAD_COLOR), image, cv::Size(), scale, scale, cv::INTER_AREA );

    if (image.empty()) throw new std::runtime_error("imread path failure : " + path );
    viewpoint->setImage(image);
    viewpoint->setImageDimension(image.cols, image.rows);

    viewpoint->position = info.position;
    viewpoint->orientation = info.orientation;
    fileIndex++;
    return viewpoint;
}

bool ViewPointSourceWithOdometry::hasNext(){
    return fileIndex <= fileLastIndex;
}





