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

#include "framework-viewpointsource.hpp"

ViewPointSourceFs::ViewPointSourceFs(std::string imageFolder, std::string firstImage, std::string lastImage, uint32_t increment, double scale) : fileIncrement(increment), imageScale(scale) {

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

            /* display pushed file name */
            std::cout << "Pusing image " << entry.path().filename() << " ..." << std::endl;

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

std::shared_ptr<Viewpoint> ViewPointSourceFs::next(){

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

bool ViewPointSourceFs::hasNext(){

    /* Detect end of image list */
	return fileIndex < fileLastIndex;

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





