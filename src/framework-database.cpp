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

Database::Database(
    unsigned long initialBootstrap,
    double initialError,
    unsigned long initialStructure,
    double initialDisparity,
    double initialRadiusMin,
    double initialRadiusMax
){
    configBootstrap=initialBootstrap;
    configError=initialError;
    configStructure=initialStructure;
    configDisparity=initialDisparity;
    configRadiusMin=initialRadiusMin;
    configRadiusMax=initialRadiusMax;
}

int Database::getViewpointCount(){
    return viewpoints.size();
}

double Database::getConfigError(){
    return configError;
}

double Database::getError(){
    double maxValue(0.);
    for(auto & element: viewpoints){
        if (element->getDisparityMean()>maxValue){
            maxValue=element->getDisparityMean();
        }
    }
    return( maxValue );
}

void Database::addViewpoint(std::shared_ptr<Viewpoint> viewpoint){
	viewpoint->setIndex(viewpoints.size());
	if(viewpoint->getIndex() > 0) transforms.push_back(std::make_shared<Transform>());
	viewpoints.push_back(viewpoint);
}

void Database::computeModels(){
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeModel();
        }
    }
}

void Database::computeCorrelations(){
    for(auto & element: transforms){
        element->resetCorrelation();
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeCorrelation(transforms);
        }
    }
}

void Database::computeCentroids(){
    for(auto & element: transforms){
        element->resetCentroid();
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeCentroid(transforms);
        }
    }
    for(auto & element: transforms){
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
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeOptimalPosition();
        }
    }
}

void Database::computeRadii(){
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeRadius(viewpoints);
        }
    }
}

void Database::computeStatistics(){
    for(auto & element: viewpoints){
        element->resetStatistics();
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeStatisticsMean();
        }
    }
    for(auto & element:viewpoints){
        element->computeStatisticsMean();
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            element->computeStatisticsSD();
        }
    }
    for(auto & element: viewpoints){
        element->computeStatisticsSD();
    }
    for(unsigned int i(0); i<transforms.size(); i++){
        viewpoints[i]->setReferenceDistance((*transforms[i]->getTranslation()).norm());
    }
    viewpoints.back()->setReferenceDistance((*transforms.back()->getTranslation()).norm());
}

//Issue index of following elements will be modifed, can't be use in computeFilter as this
//void Database::deleteAndUnlinkStructure(int i){
//	for(auto f : *structures[i]->getFeatures()){
//		f->structure = NULL;
//	}
//	std::swap(structures[i],structures[structures.size()-1]);
//  structures.resize(structures.size()-1);
//}

void Database::computeFilters(){
    unsigned int i(0);
    unsigned int j(structures.size());
    while (i < j){
        if(structures[i]->getFeaturesCount()>=configStructure){
        if (structures[i]->computeFilter(configDisparity,configRadiusMin,configRadiusMax)==false){
        	for(auto f : *structures[i]->getFeatures()){
        		f->structure = NULL;
        	}
            std::swap(structures[i],structures[--j]);
        } else {
            i++;
        }
        }else{i++;}
    }
    structures.resize(j);
}

void Database::extrapolateViewpoint(Viewpoint * pushedViewpoint){
    auto viewpointCount(viewpoints.size());
    if(viewpointCount<=configBootstrap){
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
    if (viewpoints.size()<=configBootstrap){
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
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            Eigen::Vector3d * position(element->getPosition());
            exportStream << (*position)(0) << " " << (*position)(1) << " " << (*position)(2) << " 255 0 0" << std::endl;
        }
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
    for(auto & element: viewpoints){
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

// Note : this function does not respect encapsulation (development function)
void Database::_exportState(std::string path, int major, int iter){
    int vpcount(255/viewpoints.size());
    std::fstream stream;
    stream.open( path + "/" + std::to_string(major) + "_" + std::to_string(iter) + "_model.xyz", std::ios::out );
    if (stream.is_open()==false){
        std::cerr << "unable to create state exportation file" << std::endl;
        return;
    }
    for(auto & element: viewpoints){
        stream << element->position(0) << " "
               << element->position(1) << " "
               << element->position(2) << " 0 0 255" << std::endl;
    }
    for(auto & element: structures){
        if(element->getFeaturesCount()>=configStructure){
            stream << element->position(0) << " "
                   << element->position(1) << " "
                   << element->position(2) << " 255 0 255" << std::endl;
            for(unsigned int j(0); j<element->features.size(); j++){
                Eigen::Matrix3d matrix(*element->features[j]->getViewpoint()->getOrientation());
                Eigen::Vector3d vector(*element->features[j]->getViewpoint()->getPosition());
                Eigen::Vector3d Position(matrix*(element->features[j]->direction*element->features[j]->radius)+vector);
                stream << Position(0) << " "
                       << Position(1) << " "
                       << Position(2) << " 255 " << j*vpcount << " 0" << std::endl;
            }
        }
    }
    stream.close();
}

