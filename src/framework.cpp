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

#include "framework.hpp"

int main(int argc, char ** argv){

    //
    //  Usage check
    //

    // Check usage
    if ( argc != 2 ) {
        // Display minimal help
        std::cerr << "Wrong usage" << std::endl << "Usage : sfs-framework .../config_file.yaml" << std::endl;
    }

    //
    //  Framework initialisation
    //

    // Configuration file importation
    YAML::Node config = YAML::LoadFile(argv[1]);

    // Framework main structure initialisation
    Database database(
        config["algorithm"]["error"].as<double>(),
        config["algorithm"]["disparity"].as<double>(),
        config["algorithm"]["radius"].as<double>(),
        config["matching"]["range"].as<unsigned int>()
    );

    // Thread pool initialisation
    ThreadPool threadpool(2);

    // Framework front-end
    Frontend * frontend(nullptr);

    // Pipeline iterations
    int loopMajor(1);
    int loopMinor(0);

    // Algorithm loop
    bool loopFlag(true);
    bool loopTrig(false);
    long loopState=0;

    // Algorithm error
    double loopPError(1.);
    double pushPError(0.);
    double loopDError(1.);
    double pushDError(0.);

    // Algorithm filtering
    unsigned int pushFilter(0);

    // development feature - begin
    bool inlinerEnabled = false;
    // development feature - end

    //
    //  Framework exportation
    //

    // Create exportation directories
    create_directories(config["export"]["path"].as<std::string>(), config["frontend"]["type"].as<std::string>());

    //
    //  Framework front-end
    //

    // Switch on front-end : odometry (IMAGE), densification (DENSE), synthetic model (CLOUDPOINT)
    if(config["frontend"]["type"].as<std::string>() == "sparse"){

        auto fs = config["frontend"]["source"];
        ViewPointSource *source = NULL;
        auto sourceType = fs["type"].as<std::string>();
        if(sourceType == "FOLDER") {
            std::string firstFile =  fs["first"].IsDefined() ? fs["first"].as<std::string>() : "";
            std::string lastFile = fs["last"].IsDefined() ? fs["last"].as<std::string>() : "";
            source = new ViewPointSourceFs(fs["path"].as<std::string>(), fs["scale"].as<double>(), firstFile, lastFile, fs["inc"].as<uint32_t>());
        }
        auto mask = cv::imread(fs["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
        if(fs["scale"].IsDefined()){
            auto scale = fs["scale"].as<double>();
            cv::resize(mask, mask, cv::Size(), scale, scale, cv::INTER_AREA );
        }
        frontend = new FrontendPicture(source, mask, &threadpool, &database);

    } else
    if(config["frontend"]["type"].as<std::string>() == "DENSE"){

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

    } else
    if(config["frontend"]["type"].as<std::string>() == "CLOUDPOINT"){

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

    //
    //  Framework main loop
    //

    // framework loop
    while(true){

        // Query image from source
        if(!frontend->next()){
            // drop the pushed image
            continue;
        }

        // development feature - begin
        database._sanityCheck(inlinerEnabled);
        // development feature - end

        // Wait bootstrap image count
        if(database.getBootstrap()){
            // avoid optimisation
            exitRelease();
            continue;
        }

        // development feature - begin
        if(config["debug"].IsDefined()){
            if(config["debug"]["structureImageDump"].IsDefined()){
                for(auto viewpoint : database.viewpoints){
                    auto image = database.viewpointStructuralImage(viewpoint.get(), 0);
                    auto folder = config["export"]["path"].as<std::string>() + "/viewpointStructuresImages";
                    auto path = folder + "/" + std::to_string(viewpoint->index) + "_" + std::to_string(loopMajor) + "a.png";
                    fs::create_directories(folder);
                    cv::imwrite(path, image);
                }
            }
        }
        // development feature - end

        // Perpare structure features - sort by viewpoint index order
        database.prepareFeature();

        // Prepare structure vector - type-based segment sort
        database.prepareStructure();

        // development feature - begin
        database._sanityCheckFeatureOrder();
        // development feature - end

        // Reset algorithm loop
        loopFlag=true;
        loopMinor=0;

        // development feature - begin
        //database._exportMatchDistribution(config["export"]["path"].as<std::string>(),loopMajor,"front");
        // development feature - end

        // development feature - begin
        //std::cerr << "Distribution " << database.sortStructTypeA << " " << database.sortStructTypeB << std::endl;
        // development feature - end

        // Algorithm state loop
        while ( loopFlag == true ) {

            // reset algorithm
            pushPError=-1.;
            pushDError=-1.;

            // algorithm optimisation loop
            while ( loopFlag == true ) {

                // Algorithm core
                database.computeModels(loopState);
                database.computeCentroids(loopState);
                database.computeCorrelations(loopState);
                database.computePoses(loopState);
                database.computeFrames(loopState);
                database.computeOriented(loopState);
                database.computeOptimals(loopState);
                database.computeRadii(loopState);

                // Push amount of structures
                pushFilter = database.structures.size();

                // Stability filtering - radius positivity
                database.filterRadialPositivity(loopState);

                // Statistics computation on disparity
                database.computeDisparityStatistics(loopState);

                // Filtering processes
                database.filterDisparity(loopState);

                // Extarct error values
                loopPError = database.getPError();
                loopDError = database.getDError();

                // development feature - begin
                if(std::isnan(loopPError)){
                    std::cerr<<"Crash on NaN"<<std::endl; exit(1);
                }
                if(std::isnan(loopDError)){
                    std::cerr<<"Crash on NaN"<<std::endl; exit(1);
                }
                // development feature - end

                // Display information
                std::cout << "step : " << std::setw(6) << loopMajor << " | iter : " << loopMinor << " | state " << loopState << " | error : (" << loopPError << ", " << loopDError << ")" << std::endl;

                // development feature - begin
                //database._sanityCheckStructure();
                // development feature - end

                // development feature - begin
                //database._exportState(config["export"]["path"].as<std::string>(),loopMajor,loopMinor);
                // development feature - end

                // Optimisation step condition
                loopTrig=false;
                if((database.getCheckError(loopPError, pushPError)) && (database.getCheckError(loopDError, pushDError))){
                    loopTrig=true;
                }
                if(loopMinor>DB_LOOP_MAXITER){
                    loopTrig=true;
                }

                // Check step condition
                if (loopTrig==true) {

                    // Filtering process
                    database.filterRadialLimitation(loopState);

                    // Optimisation end condition
                    if((database.structures.size()==pushFilter)||(loopMinor>DB_LOOP_MAXITER)){
                        loopFlag = false;
                    }

                }

                pushPError = loopPError;
                pushDError = loopDError;

                // Update minor iterator
                loopMinor ++;

            }

            // State loop management
            if((loopState==DB_LOOP_MODE_BOOT)||(loopState==DB_LOOP_MODE_FULL)){

                // Update loop mode
                loopState=DB_LOOP_MODE_LAST;

            } else
            if (loopState==DB_LOOP_MODE_LAST){

                // Update loop mode
                loopState=DB_LOOP_MODE_FULL;
                //loopState=DB_LOOP_MODE_LAST;

                // Reset loop flag (continue optimisation)
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
                cv::waitKey(0); //Wait 100 ms give opencv the time to display the GUI
            }


            if(config["debug"]["structureImageDump"].IsDefined()){
                allowDeallocateImages = false;
                for(auto viewpoint : database.viewpoints){
                    auto image = database.viewpointStructuralImage(viewpoint.get(), 0);
                    auto folder = config["export"]["path"].as<std::string>() + "/viewpointStructuresImages";
                    auto path = folder + "/" + std::to_string(viewpoint->index) + "_" + std::to_string(loopMajor) + "b.png";
                    fs::create_directories(folder);
                    cv::imwrite(path, image);
                }
            }
        }

        // check viewpoint stack
        if ( database.viewpoints.size() > database.configMatchRange ) {
            // release viewpoint image memory
            database.viewpoints[database.viewpoints.size()-database.configMatchRange]->releaseImage();
        }

        //if(allowDeallocateImages && database.viewpoints.size() >= 4) {
            //database.viewpoints[database.viewpoints.size()-4]->getImage()->deallocate(); //TODO As currently we aren't using the image, we can just throw it aways to avoid memory overflow.
            //database.viewpoints[database.viewpoints.size()-4]->releaseImage();
        //}

        // Major iteration exportation : model, odometry and transformation
        database.exportStructure     (config["export"]["path"].as<std::string>(),loopMajor);
        database.exportPosition      (config["export"]["path"].as<std::string>(),loopMajor);
        database.exportTransformation(config["export"]["path"].as<std::string>(),loopMajor);

        // update major iterator
        loopMajor ++;

        exitRelease();
    }

    // system message
    return 0;

}

