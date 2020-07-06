/*
 *  sfs-framework
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Charles Papon - charles.papon.90@gmail.com
 *      Copyright (c) 2019-2020 DHLAB, EPFL & HES-SO Valais-Wallis
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

#include "framework-source.hpp"

//
//  Sparse source
//

SourceSparse::SourceSparse(std::string imageFolder, std::string firstImage, std::string lastImage, uint32_t increment, double scale) :

    Source(
        increment, 
        scale
    )

{

    /* Image path */
    fs::path imagePath(fs::u8path(imageFolder));

    /* Image extension */
    fs::path imageExtension;

    /* Detect valid image in folder */
    for(auto & entry: fs::directory_iterator(imagePath)){

        /* Extract image extension */
        imageExtension = fs::path(entry.path().u8string()).extension();

        /* Selection based on file extension */
        if((imageExtension==".bmp")||(imageExtension==".jpg")||(imageExtension==".png")||(imageExtension==".tif")){

            /* Display pushed file name */
            std::cout << "Pushing image " << entry.path().filename() << " ..." << std::endl;

            /* Push image in the list */
            files.push_back(entry.path().u8string());

        }

    }

    /* sort files list alphabetically */
    std::sort(files.begin(), files.end());

    /* check for file range boundary */
    if(firstImage.empty()){

        /* initialise file index */
        fileIndex = 0;

    }else{

        /* detect index of specified initial file */
        if((fileIndex = findInVector(files, firstImage).second)<0){

            /* display warning */
            std::cerr << "Warning : unable to locate specified frist file. Using first file in the list" << std::endl;

            /* initialise file index */
            fileIndex = 0;

        }

    }

    /* check for file range boundary */
    if(lastImage.empty()){

        /* initialise last index */
        fileLastIndex = files.size();

    }else{

        /* detect index of specified initial file */
        if((fileIndex = findInVector(files, lastImage).second)<0){

            /* display warning */
            std::cerr << "Warning : unable to locate specified last file. Using last file in the list" << std::endl;

            /* initialise last index */
            fileLastIndex = files.size();

        }

    }

}

std::shared_ptr<Viewpoint> SourceSparse::next(){

    /* Create new viewpoint instance */
    std::shared_ptr<Viewpoint> pushViewpoint = std::make_shared<Viewpoint>();

    /* import and scale image */
    if(pushViewpoint->setImage(files[fileIndex], imageScale)==false){

        /* send critical message */
        throw std::runtime_error("Error : unable to import image " + files[fileIndex]);

    }

    /* assign viewpoint uid (filename) */
    pushViewpoint->uid = fs::path(files[fileIndex]).filename();

    /* update file index */
    fileIndex += fileIncrement;

    /* return created viewpoint */
    return pushViewpoint;

}

bool SourceSparse::hasNext(){

    /* Detect end of image list */
	return fileIndex < fileLastIndex;

}

//
//  Dense source
//

SourceDense::SourceDense(std::string imageFolder, std::string transformationFile, std::string firstImage, std::string lastImage, int increment, double scale) : 

    Source(
        increment, 
        scale
    ), 
    pictureFolder(imageFolder) 

{

    /* Transformation input stream */
    std::ifstream transformationStream(transformationFile);

    /* Viewpoint transformation structure */
    ViewPointInfo importTransformation;

    /* Initialise index and boundary */
    fileIndex = -1;
    fileLastIndex = -1;

    /* Check stream */
    if(transformationStream.is_open()==false){

        /* Send critical message */
        throw std::runtime_error("Error : unable to read transformation file");

    }

    /* Reading transformation */
    while(transformationStream.eof()==false) {

        /* Import viewpoint uid (filename) */
        if(transformationStream >> importTransformation.fileName) {

            /* Display imported file name */
            std::cout << "Pushing image " << importTransformation.fileName << " ..." << std::endl;

            /* Import position */
            transformationStream >> importTransformation.position(0);
            transformationStream >> importTransformation.position(1);
            transformationStream >> importTransformation.position(2);

            /* Import orientation */
            transformationStream >> importTransformation.orientation(0,0);
            transformationStream >> importTransformation.orientation(0,1);
            transformationStream >> importTransformation.orientation(0,2);
            transformationStream >> importTransformation.orientation(1,0);
            transformationStream >> importTransformation.orientation(1,1);
            transformationStream >> importTransformation.orientation(1,2);
            transformationStream >> importTransformation.orientation(2,0);
            transformationStream >> importTransformation.orientation(2,1);
            transformationStream >> importTransformation.orientation(2,2);

            /* Push transformation on list */
            list.push_back(importTransformation);

            /* Detect first image boundary */
            if(importTransformation.fileName==firstImage) {

                /* Update index */
                fileIndex = list.size() - 1;

            }

            /* Detect last image boundary */
            if(importTransformation.fileName==lastImage) {

                /* Update last index */
                fileLastIndex = list.size();

            }

        }

    }

    /* Close transformation stream */
    transformationStream.close();

    /* Check detected boundary */
    if(fileIndex<0){

        /* Check specified boundary */
        if(firstImage.empty()==false) {

            /* display warning */
            std::cerr << "Warning : unable to locate specified frist file. Using first file in the list" << std::endl;

        }

        /* initialise file index */
        fileIndex = 0;

    }

    /* Check detected boundary */
    if(fileLastIndex<0){

        /* Check specified boundary */
        if(lastImage.empty()==false){

            /* Display warning */
            std::cerr << "Warning : unable to locate specified last file. Using last file in the list" << std::endl;

        }

        /* Initialise last index */
        fileLastIndex = list.size();

    }

}

std::shared_ptr<Viewpoint> SourceDense::next(){

    /* Create new viewpoint instance */
    std::shared_ptr<Viewpoint> pushViewpoint = std::make_shared<Viewpoint>();

    /* import and scale image */
    if(pushViewpoint->setImage(pictureFolder + "/" + list[fileIndex].fileName, imageScale)==false){

        /* send critical message */
        throw std::runtime_error("Error : unable to import image " + list[fileIndex].fileName);

    }
    
    /* assign viewpoint uid (filename) */
    pushViewpoint->uid = list[fileIndex].fileName;

    /* assign viewpoint position */
    pushViewpoint->position = list[fileIndex].position;

    /* assign viewpoint orientation */
    pushViewpoint->orientation = list[fileIndex].orientation;

    /* update file index */
    fileIndex += fileIncrement;

    /* return created viewpoint */
    return pushViewpoint;

}

bool SourceDense::hasNext(){

    /* Detect end of image list */
    return fileIndex < fileLastIndex;

}

