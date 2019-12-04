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
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[]){
    //profile("boot");
    assert(argc == 2);
    std::cout << "Hello world!" << std::endl;

    YAML::Node config = YAML::LoadFile(argv[1]);

    auto database = Database(
        config["algorithm"]["error"].as<double>(),
        config["algorithm"]["disparity"].as<double>(),
        config["algorithm"]["radius"].as<double>()
    );
    ThreadPool threadpool(8);

    bool inlinerEnabled = false;
    Frontend *frontend = NULL;
    auto frontendType = config["frontend"]["type"].as<std::string>();
    if(frontendType == "IMAGE"){
        auto fs = config["frontend"]["source"];
        ViewPointSource *source = NULL;
        auto sourceType = fs["type"].as<std::string>();
        if(sourceType == "FOLDER") {
            std::string firstFile =  fs["first"].IsDefined() ? fs["first"].as<std::string>() : "";
            std::string lastFile = fs["last"].IsDefined() ? fs["last"].as<std::string>() : "";
            source = new ViewPointSourceFs(fs["path"].as<std::string>(), fs["scale"].as<double>(), firstFile, lastFile);
        }
        auto mask = cv::imread(fs["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
        if(fs["scale"].IsDefined()){
            auto scale = fs["scale"].as<double>();
            cv::resize(mask, mask, cv::Size(), scale, scale, cv::INTER_AREA );
        }
        frontend = new FrontendPicture(source, mask, &threadpool, &database);
    }

    if(frontendType == "DENSE"){
        auto fs = config["frontend"]["source"];
        ViewPointSource *source = NULL;
        auto sourceType = fs["type"].as<std::string>();
        if(sourceType == "ODOMETRY") {
            std::string firstFile =  fs["first"].IsDefined() ? fs["first"].as<std::string>() : "";
            std::string lastFile = fs["last"].IsDefined() ? fs["last"].as<std::string>() : "";
            source = new ViewPointSourceWithOdometry(
                fs["odometryFile"].as<std::string>(),
                fs["pictureFolder"].as<std::string>(),
                fs["scale"].as<double>(),
                firstFile,
                lastFile
            );
        }
        auto mask = cv::imread(fs["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
        if(fs["scale"].IsDefined()){
            auto scale = fs["scale"].as<double>();
            cv::resize(mask, mask, cv::Size(), scale, scale, cv::INTER_AREA );
        }
        frontend = new FrontendDense(source, mask, &database, config["frontend"]["ofCacheFolder"].as<std::string>());
    }


    if(frontendType == "CLOUDPOINT"){
        auto fn = config["frontend"];
        frontend = new FrontendCloudpoint(
                &database, fn["model"].as<std::string>(),
                fn["odometry"].as<std::string>(),
                fn["distanceMax"].as<double>(),
                fn["badMatchRate"].as<double>(),
                fn["baseNoise"].as<double>(),
                fn["badMatchNoise"].as<double>());
        inlinerEnabled = true;
    }

    // pipeline iterations
    int loopMajor(1);
    int loopMinor(0);

    // algorithm loop
    bool loopFlag(true);
    long loopState=0;

    // algorithm error
    double loopError(1.);
    double pushError(0.);

    // pipeline loop
    while(true){

        //
        // image stream front-end
        //

        // query image from source
        if(!frontend->next()){
            // drop the pushed image
            continue;
        }

        // development feature - begin
        database._sanityCheck(inlinerEnabled);
        // development feature - end

        //
        // geometry estimation solver
        //

        // wait bootstrap image count
        if(database.getBootstrap()){
            // avoid optimisation
            exitRelease();
            continue;
        }

        // prepare structure vector - type-based segment sort
        database.prepareStructure();

        // reset algorithm loop
        loopFlag=true;
        loopMinor=0;

        // development feature - begin
        //database._exportMatchDistribution(config["export"]["path"].as<std::string>(),loopMajor,"front");
        // development feature - end

        // algorithm state loop
        while ( loopFlag == true ) {

            // reset algorithm
            loopError=0.;
            pushError=1.;

            // algorithm optimisation loop
            while ( loopFlag == true ) {

                // algorithm core
                database.computeModels(loopState);
                database.computeCentroids(loopState);
                database.computeCorrelations(loopState);
                database.computePoses(loopState);
                database.computeFrames();
                database.computeOptimals(loopState);
                database.computeRadii(loopState);

                database.computeFiltersRadialClamp(loopState);
                //database.computeStatistics(loopState,&Feature::getRadius);
                //database.computeFiltersRadialStatistics(loopState);
                //database.computeStatistics(loopState,&Feature::getDisparity);
                //database.computeFiltersDisparityStatistics(loopState);
                database.computeStatistics(loopState,&Feature::getDisparity);

                // development feature - begin
                //database._sanityCheckStructure();
                // development feature - end

                // development feature - begin
                //database._exportState(config["export"]["path"].as<std::string>(),loopMajor,loopMinor);
                // development feature - end

                // get error value
                loopError = database.getError();

                // update minor iterator
                loopMinor ++;

                // display information
                std::cout << "step : " << std::setw(6) << loopMajor << " | iteration : " << std::setw(3) << loopMinor << " | state : " << loopState << " | error : " << loopError << std::endl;

                // optimisation loop management
                if((fabs(loopError - pushError) < database.getConfigError()) || std::isnan(loopError)) {
                    if(loopState==DB_LOOP_MODE_FULL){
                        loopFlag=false;
                    }else{
                        unsigned int pushCount(database.structures.size());
                        database.computeFiltersDisparityStatistics(loopState);
                        if(database.structures.size()==pushCount){
                            loopFlag = false;
                        }else{
                            pushError=1.;
                        }
                    }
                } else {
                    pushError=loopError;
                }

            }

            // state loop management
            if( (loopState==DB_LOOP_MODE_BOOT) ||
                (loopState==DB_LOOP_MODE_FULL) ){
                loopState=DB_LOOP_MODE_LAST;
            } else
            if (loopState==DB_LOOP_MODE_LAST){
                loopState=DB_LOOP_MODE_FULL;
                loopFlag=true;
            }

        }

        // development feature - begin
        database._sanityCheck(inlinerEnabled);
        // development feature - end

        bool allowDeallocateImages = true;

        if(config["debug"].IsDefined()){
            auto lastViewPointGui = config["debug"]["lastViewPointGui"];
            if(lastViewPointGui.IsDefined() && database.viewpoints.back()->getImage()->cols != 0){
                database._displayViewpointStructures(database.viewpoints.back().get(), lastViewPointGui["structureSizeMin"].as<int>());
                cv::waitKey(100); //Wait 100 ms give opencv the time to display the GUI
            }


            if(config["debug"]["structureImageDump"].IsDefined()){
                allowDeallocateImages = false;
                for(auto viewpoint : database.viewpoints){
                    auto image = database.viewpointStructuralImage(viewpoint.get(), 0);
                    auto folder = config["export"]["path"].as<std::string>() + "/viewpointStructuresImages";
                    auto path = folder + "/" + std::to_string(viewpoint->index) + "_" + std::to_string(loopMajor) + ".png";
                    fs::create_directories(folder);
                    cv::imwrite(path, image);
                }
            }
        }

        if(allowDeallocateImages) database.viewpoints.back()->getImage()->deallocate(); //TODO As currently we aren't using the image, we can just throw it aways to avoid memory overflow.

        // major iteration exportation : model and odometry
        database.exportModel         (config["export"]["path"].as<std::string>(),loopMajor);
        database.exportOdometry      (config["export"]["path"].as<std::string>(),loopMajor);
        database.exportTransformation(config["export"]["path"].as<std::string>(),loopMajor);

        // update major iterator
        loopMajor ++;

        exitRelease();
    }

    // system message
    return 0;

}

