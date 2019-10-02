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

#include "framework-database.hpp"

void Database::setPath(std::string recordPath, std::string modelPath){
    ios.setRecordPath( recordPath );
    ios.setModelPath( modelPath );
}

void Database::computeModels(){
    for(unsigned int i(0); i<viewpoints.size(); i++){
        viewpoints[i]->computeModel();
        viewpoints[i]->computeCentroid();
    }
}

void Database::computeCorrelations(){
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->resetCorrelation();
    }
    for(unsigned int i(0); i<structures.size(); i++){
        structures[i]->computeCorrelation(viewpoints,transforms);
    }
}

void Database::computePoses(){
    for(unsigned int i(1); i<transforms.size(); i++){
        transforms[i]->computePose(viewpoints[i-1].get(),viewpoints[i].get());
    }
}

void Database::computeFrame(){
    viewpoints[0]->resetFrame();
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->computeFrame(viewpoints[i].get(),viewpoints[i+1].get());
    }
}

void Database::computeRadius(){
    for(unsigned int i(0); i<structures.size(); i++){
        structures[i]->computeRadius(viewpoints);
    }
}

//Issue index of following elements will be modifed, can't be use in computeFilter as this
//void Database::deleteAndUnlinkStructure(int i){
//	for(auto f : *structures[i]->getFeatures()){
//		f->structure = NULL;
//	}
//	std::swap(structures[i],structures[structures.size()-1]);
//  structures.resize(structures.size()-1);
//}

void Database::computeFilter(double dispTolerence, double triTolerence){
    unsigned int i(0);
    unsigned int j(structures.size());
    while ( i<j ){
        if ( structures[i]->computeFilter( &viewpoints, dispTolerence, triTolerence ) == false ){
        	for(auto f : *structures[i]->getFeatures()){
        		f->structure = NULL;
        	}
            std::swap(structures[i],structures[--j]);
        } else {
            i++;
        }
    }
    structures.resize(j);
}

double Database::computeError(){
    double error(0.);
    double candidate(0.);
    for(unsigned int i(0); i<structures.size(); i++){
        candidate=structures[i]->getDisparity();
        if ( candidate>error ) error=candidate;
    }
    return error;
}

void Database::exportModel(){
    std::string * exportPath(ios.getModelPath());
    std::fstream exportStream;
    exportStream.open((*exportPath)+"/model.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create model exportation file" << std::endl;
        return;
    }
    for(unsigned int i(0); i<structures.size(); i++){
        Eigen::Vector3d * position(structures[i]->getPosition());
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << std::endl;
    }
    exportStream.close();
}

void Database::exportOdometry(){
    std::string * exportPath(ios.getModelPath());
    std::fstream exportStream;
    exportStream.open((*exportPath)+"/odometry.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create odometry exportation file" << std::endl;
        return;
    }
    for(unsigned int i(0); i<viewpoints.size(); i++){
        Eigen::Vector3d * position(viewpoints[i]->getPosition());
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << std::endl;
    }
    exportStream.close();
}

static cv::Point f2i(Eigen::Vector2f value){
	return cv::Point(value[0],value[1]);
}


//Do  cv::waitKey(0); if you want to stop after it.
void Database::displayViewpointStructures(Viewpoint *viewpoint){
	cv::RNG rng(12345);
	cv::Rect myROI(0, 0, viewpoint->getImage()->cols, viewpoint->getImage()->rows);
	cv::Mat res(myROI.width,myROI.height, CV_8UC3, cv::Scalar(0,0,0));
	res = *viewpoint->getImage();
	for(auto f : *viewpoint->getFeatures()){
		if(!f.structure) continue;
		cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

		auto features = f.structure->getFeatures();
		for(uint32_t idx = 1;idx < features->size();idx++){
			cv::line(res, f2i((*features)[idx-1]->position),  f2i((*features)[idx]->position), color, 2);
		}
	}

	cv::namedWindow( "miaou", cv::WINDOW_KEEPRATIO );
	imshow( "miaou", res);
}
