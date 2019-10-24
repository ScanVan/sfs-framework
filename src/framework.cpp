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

int main(int argc, char *argv[]){
    //profile("boot");
    assert(argc == 2);
    std::cout << "Hello world!" << std::endl;

    YAML::Node config = YAML::LoadFile(argv[1]);

    auto database = Database(
        config["algorithm"]["error"].as<double>(),
        config["algorithm"]["structure"].as<unsigned long>(),
        config["algorithm"]["disparity"].as<double>(),
        config["algorithm"]["radius"].as<double>()
    );
    ThreadPool threadpool(8);

    bool inlinerEnabled = false;
    Frontend *frontend = NULL;
    auto frontendType = config["frontend"]["type"].as<std::string>();
    if(frontendType == "IMAGE"){
        ViewPointSource *source = NULL;
        auto sourceType = config["frontend"]["source"]["type"].as<std::string>();
        if(sourceType == "FOLDER") {
            source = new ViewPointSourceFs(config["frontend"]["source"]["path"].as<std::string>(), config["frontend"]["source"]["scale"].as<double>());
        }
        auto mask = cv::imread(config["frontend"]["source"]["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
        if(config["frontend"]["source"]["scale"].IsDefined()){
            auto scale = config["frontend"]["source"]["scale"].as<double>();
            cv::resize(mask, mask, cv::Size(), scale, scale, cv::INTER_AREA );
        }
        frontend = new FrontendPicture(source, mask, &threadpool, &database);
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

    // pipeline major iteration
    int loopMajor(1);

    // pipeline minor iteration
    int loopMinor( 0 );

    // algorithm variables
    double loopError( 1. );
    double pushError( 0. );
    bool loopFlag( true );

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
            continue;
        }

        // reset algorithm variables
        loopError=0.;
        pushError=1.;
        loopFlag=true;
        loopMinor=0;

        // development feature - begin
        //database._exportMatchDistribution(config["export"]["path"].as<std::string>(),loopMajor,"front");
        // development feature - end

        // algorithm loop
        while ( loopFlag == true ) {

            // algorithm core
            database.computeModels();
            database.computeCentroids();
            database.computeCorrelations();
            database.computePoses();
            database.computeFrames();
            database.computeOptimals();
            database.computeRadii();
            database.computeStatistics();
            database.computeFilters();

            // development feature - begin
            //database._exportState(config["export"]["path"].as<std::string>(),loopMajor,loopMinor);
            // development feature - end

            // algorithm error management
            loopError = database.getError();
            if(fabs( loopError - pushError ) < database.getConfigError()) {
                loopFlag = false;
            } else {
                pushError=loopError;
            }

            // update minor iterator
            loopMinor ++;

            // display information
            std::cout << "step : " << std::setw(6) << loopMajor << " | iteration : " << std::setw(3) << loopMinor << " | error : " << loopError << std::endl;

        }

        // development feature - begin
        database._sanityCheck(inlinerEnabled);
        // development feature - end

        if(config["debug"].IsDefined()){
            auto lastViewPointGui = config["debug"]["lastViewPointGui"];
            if(lastViewPointGui.IsDefined()){
                database._displayViewpointStructures(database.viewpoints.back().get(), lastViewPointGui["structureSizeMin"].as<int>());
                cv::waitKey(100); //Wait 100 ms give opencv the time to display the GUI
            }
        }
        database.viewpoints.back()->getImage()->deallocate(); //TODO As currently we aren't using the image, we can just throw it aways to avoid memory overflow.


        // major iteration exportation : model and odometry
        database.exportModel   (config["export"]["path"].as<std::string>(),loopMajor);
        database.exportOdometry(config["export"]["path"].as<std::string>(),loopMajor);

        // update major iterator
        loopMajor ++;

    }

    // system message
    return 0;

}

