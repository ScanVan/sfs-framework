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

int Database::getViewpointCount(){
    return viewpoints.size();
}

double Database::getError(){
    return disparityMean;
}

void Database::addViewpoint(std::shared_ptr<Viewpoint> viewpoint){
	viewpoint->setIndex(viewpoints.size());
	if(viewpoint->getIndex() > 0) transforms.push_back(std::make_shared<Transform>());
	viewpoints.push_back(viewpoint);
}

void Database::computeModels(){
    for(auto element: structures){
        element->computeModel();
    }
}

void Database::computeCorrelations(){
    for(auto element: transforms){
        element->resetCorrelation();
    }
    for(auto element: structures){
        element->computeCorrelation(transforms);
    }
}

void Database::computeCentroids(){
    for(auto element: transforms){
        element->resetCentroid();
    }
    for(auto element: structures){
        element->computeCentroid(transforms);
    }
    for(auto element: transforms){
        element->computeCentroid();
    }
}

void Database::computePoses(){
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->computePose(viewpoints[i].get(),viewpoints[i+1].get());
    }
}

void Database::computeFrames(){
    viewpoints[0]->resetFrame();
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->computeFrame(viewpoints[i].get(),viewpoints[i+1].get());
    }
}

void Database::computeOptimals(){
    for(auto element: structures){
        element->computeOptimalPosition();
    }
}

void Database::computeRadii(){
    for(auto element: structures){
        element->computeRadius(viewpoints);
    }
}

void Database::computeStatistics(){
    unsigned int count(0);
    double component(0.);

    disparityMean=0.;
    radiusMean=0.;
    for(unsigned int i(0); i<structures.size(); i++){
        for(unsigned int j(0); j<structures[i]->getFeatureCount(); j++){
            disparityMean+=structures[i]->getDisparity(j);
            radiusMean+=structures[i]->getRadius(j);
            count++;
        }
    }
    disparityMean/=double(count);
    radiusMean/=double(count);

    disparitySD=0.;
    radiusSD=0.;
    for(unsigned int i(0); i<structures.size(); i++){
        for(unsigned int j(0); j<structures[i]->getFeatureCount(); j++){
            component=structures[i]->getDisparity(j)-disparityMean;
            disparitySD+=component*component;
            component=structures[i]->getRadius(j)-radiusMean;
            radiusSD+=component*component;
        }
    }
    disparitySD=std::sqrt(disparitySD/double(count-1));
    radiusSD=std::sqrt(radiusSD/double(count-1));
}


//Issue index of following elements will be modifed, can't be use in computeFilter as this
//void Database::deleteAndUnlinkStructure(int i){
//	for(auto f : *structures[i]->getFeatures()){
//		f->structure = NULL;
//	}
//	std::swap(structures[i],structures[structures.size()-1]);
//  structures.resize(structures.size()-1);
//}

void Database::computeFilters(double dispTolerence, double radTolerence){
    unsigned int i(0);
    unsigned int j(structures.size());

    double dispFilter(disparitySD*dispTolerence);
    double radFilter(radiusSD*radTolerence);

    while (i < j){
        //if (structures[i]->computeFilter(disparitySD,radiusMean,radiusSD,dispTolerence,radTolerence)==false){
        if (structures[i]->computeFilter(dispFilter, radFilter, radiusMean)==false){
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

void Database::extrapolateViewpoint(Viewpoint * pushedViewpoint){
    auto viewpointCount(viewpoints.size());
    if(viewpointCount<2){
        pushedViewpoint->resetFrame();
    }else{
        Eigen::Matrix3d prevRotation((*transforms[viewpointCount-2]->getRotation()).transpose());
        Eigen::Vector3d prevTranslation(prevRotation*(*transforms[viewpointCount-2]->getTranslation()));
        pushedViewpoint->setPose(
            (*viewpoints[viewpointCount-1]->getOrientation())*prevRotation,
            (*viewpoints[viewpointCount-1]->getPosition())-prevTranslation
        );
    }
}

void Database::extrapolateStructure(){
    if (viewpoints.size()<=2){
        return;
    }
    for(auto element: structures){
        element->extrapolate(viewpoints);
    }
}

void Database::exportModel(std::string path, unsigned int major){
    std::fstream exportStream;
    exportStream.open(path+"/dev/"+std::to_string(major)+"_structure.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create model exportation file" << std::endl;
        return;
    }
    for(auto element: structures){
        Eigen::Vector3d * position(element->getPosition());
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " 255 0 0" << std::endl;
    }
    exportStream.close();
}

void Database::exportOdometry(std::string path, unsigned int major){
    std::fstream exportStream;
    exportStream.open(path+"/dev/"+std::to_string(major)+"_odometry.xyz",std::ios::out);
    if (exportStream.is_open() == false){
        std::cerr << "unable to create odometry exportation file" << std::endl;
        return;
    }
    for(auto element: viewpoints){
        Eigen::Vector3d * position(element->getPosition());
        exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " 255 255 255" << std::endl;
    }
    exportStream.close();
}

//
//  development related features
//

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

void Database::_exportState(std::string path, int major, int iter){
    std::fstream stream;
    int vpcount(255/viewpoints.size());
    path = path + "/" + std::to_string(major) + "_" + std::to_string(iter);
    stream.open( path + "_model.xyz", std::ios::out );
    for(unsigned int i(0); i<viewpoints.size(); i++){
        stream << viewpoints[i]->position(0) << " "
               << viewpoints[i]->position(1) << " "
               << viewpoints[i]->position(2) << " 0 0 255" << std::endl;
    }
    for(unsigned int i(0); i<structures.size(); i++){
        stream << structures[i]->position(0) << " "
               << structures[i]->position(1) << " "
               << structures[i]->position(2) << " 255 0 255" << std::endl;
        for(unsigned int j(0); j<structures[i]->features.size(); j++){
            Eigen::Matrix3d matrix(*structures[i]->features[j]->getViewpoint()->getOrientation());
            Eigen::Vector3d vector(*structures[i]->features[j]->getViewpoint()->getPosition());
            Eigen::Vector3d Position(matrix*(structures[i]->features[j]->direction*structures[i]->features[j]->radius)+vector);
            stream << Position(0) << " "
                   << Position(1) << " "
                   << Position(2) << " 255 " << j*vpcount << " 0" << std::endl;
        }

    }
    stream.close();
}

void Database::_exportMatch(std::string path){
    std::fstream stream;
    for(unsigned int i(0); i<structures.size(); i++){
        for(unsigned int j(0); j<structures[i]->features.size(); j++){
            for(unsigned int k(0); k<structures[i]->features.size(); k++){
                int jindex = structures[i]->features[j]->viewpoint->getIndex();
                int kindex = structures[i]->features[k]->viewpoint->getIndex();
                if ( jindex - kindex == 1 )
                    stream.open( path + "/match_" + std::to_string(kindex) + "_" + std::to_string(jindex), std::ios::app );
                    stream << structures[i]->features[k]->position(0) << " "
                           << structures[i]->features[k]->position(1) << " "
                           << structures[i]->features[j]->position(0) << " "
                           << structures[i]->features[j]->position(1) << std::endl;
                    stream.close();
            }
        }
    }
}

void Database::_exportInitialPair(std::string path){

    std::fstream stream, stream_[2];

    stream.open(path+"/initial_rotation",std::ios::out);
    stream << transforms[0]->rotation << std::endl;
    stream.close();

    stream.open(path+"/initial_translation",std::ios::out);
    stream << transforms[0]->translation << std::endl;
    stream.close();

    stream.open(path+"/initial_radius",std::ios::out);
    for(unsigned int i(0); i<viewpoints[0]->features.size(); i++){
        if(viewpoints[0]->features[i].structure!=NULL){
            stream << viewpoints[0]->features[i].radius << std::endl;
        }
    }
    stream.close();

    stream_[0].open(path+"/initial_dir_0",std::ios::out);
    stream_[1].open(path+"/initial_dir_1",std::ios::out);
    for(unsigned int i(0); i<structures.size(); i++){
        for(unsigned int j(0); j<structures[i]->features.size(); j++){

            stream_[structures[i]->features[j]->getViewpoint()->index]
            << structures[i]->features[j]->direction(0) << " "
            << structures[i]->features[j]->direction(1) << " "
            << structures[i]->features[j]->direction(2) << std::endl;

        }
    }
    stream_[0].close();
    stream_[1].close();

}

void Database::_exportFrame(std::string path){
    std::fstream stream;
    stream.open(path+"/frame",std::ios::out);
    for(unsigned int i(0); i<viewpoints.size(); i++){
        stream << viewpoints[i]->position(0) << " "
               << viewpoints[i]->position(1) << " "
               << viewpoints[i]->position(2) << std::endl
               << viewpoints[i]->orientation(0,0) << " "
               << viewpoints[i]->orientation(1,0) << " "
               << viewpoints[i]->orientation(2,0) << std::endl
               << viewpoints[i]->orientation(0,1) << " "
               << viewpoints[i]->orientation(1,1) << " "
               << viewpoints[i]->orientation(2,1) << std::endl
               << viewpoints[i]->orientation(0,2) << " "
               << viewpoints[i]->orientation(1,2) << " "
               << viewpoints[i]->orientation(2,2) << std::endl;
    }
    stream.close();
}
