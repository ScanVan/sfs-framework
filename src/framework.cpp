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
        // Display minimal help and exit
        std::cerr << "Wrong usage" << std::endl << "Usage : sfs-framework .../yamlConfig_file.yaml" << std::endl;
        return 1;
    }

    //
    //  Framework configuration
    //

    // Configuration file importation
    YAML::Node yamlConfig = YAML::LoadFile(argv[1]);

    // Check yaml format
    if ( yamlConfig["frontend"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file" << std::endl;
        return 1;
    } else
    if ( yamlConfig["matching"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file" << std::endl;
        return 1;
    } else
    if ( yamlConfig["algorithm"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file" << std::endl;
        return 1;
    } else
    if ( yamlConfig["export"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file" << std::endl;
        return 1;
    }

    // Instance of main yaml section
    YAML::Node yamlFrontend  = yamlConfig["frontend"];
    YAML::Node yamlMatching  = yamlConfig["matching"];
    YAML::Node yamlAlgorithm = yamlConfig["algorithm"];
    YAML::Node yamlExport    = yamlConfig["export"];

    //
    //  Framework initialisation
    //

    // Framework main structure initialisation
    Database database(
        yamlAlgorithm["error"].as<double>(),
        yamlAlgorithm["disparity"].as<double>(),
        yamlAlgorithm["radius"].as<double>(),
        yamlMatching["range"].as<unsigned int>()
    );

    // Thread pool initialisation
    ThreadPool threadpool(2);

    // Framework front-end
    Frontend * frontend(nullptr);

    // Pipeline iterations
    int loopMajor(DB_LOOP_BOOT_COUNT);
    int loopMinor(0);

    // Algorithm loop
    bool loopFlag(true);
    long loopState(DB_MODE_NULL);

    //
    //  Framework exportation
    //

    // Create exportation directories
    create_directories(yamlExport["path"].as<std::string>(), yamlFrontend["type"].as<std::string>());

    //
    //  Framework front-end
    //

    // Switch on front-end : odometry (IMAGE), densification (DENSE), synthetic model (CLOUDPOINT)
    if(yamlFrontend["type"].as<std::string>() == "sparse"){

        // Detect image list boundary
        std::string firstFile = yamlFrontend["first"].IsDefined() ? yamlFrontend["first"].as<std::string>() : "";
        std::string lastFile  = yamlFrontend["last" ].IsDefined() ? yamlFrontend["last" ].as<std::string>() : "";

        // Front-end source
        ViewPointSource * viewpointsource = new ViewPointSourceFs(
            yamlFrontend["image"].as<std::string>(), 
            firstFile,
            lastFile, 
            yamlFrontend["step"].as<uint32_t>(),
            yamlFrontend["scale"].as<double>()
        );

        // Import mask image
        cv::Mat mask = cv::imread(yamlFrontend["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);

        // Apply scale factor on mask image
        cv::resize(mask, mask, cv::Size(), yamlFrontend["scale"].as<double>(), yamlFrontend["scale"].as<double>(), cv::INTER_NEAREST );

        // Create front-end instance
        frontend = new FrontendPicture(viewpointsource, mask, &threadpool, &database);

        // Initialise algorithm state
        loopState = DB_MODE_BOOT;

    } else
    if(yamlFrontend["type"].as<std::string>() == "dense"){

        // Detect image list boundary
        std::string firstFile = yamlFrontend["first"].IsDefined() ? yamlFrontend["first"].as<std::string>() : "";
        std::string lastFile  = yamlFrontend["last" ].IsDefined() ? yamlFrontend["last" ].as<std::string>() : "";

        // Front-end source
        ViewPointSource * viewpointsource = new ViewPointSourceWithOdometry(
            yamlFrontend["image"].as<std::string>(),
            yamlExport["path"].as<std::string>() + "/sparse_transformation.dat",
            firstFile,
            lastFile,
            yamlFrontend["step"].as<uint32_t>(),
            yamlFrontend["scale"].as<double>()
        );

        // Import mask image
        cv::Mat mask = cv::imread(yamlFrontend["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);

        // Apply scale factor on mask image
        cv::resize(mask, mask, cv::Size(), yamlFrontend["scale"].as<double>(), yamlFrontend["scale"].as<double>(), cv::INTER_NEAREST );

        // Create front-end instance
        frontend = new FrontendDense(viewpointsource, mask, &database, yamlExport["path"].as<std::string>() + "/cache" );

        // Initialise algorithm state
        loopState = DB_MODE_MASS;

    }

    //
    //  Framework optimisation algorithm
    //

    // framework loop
    while(true){

        // Query image from source
        if(!frontend->next()){
            // drop the pushed image
            continue;
        }

        // Wait bootstrap image count
        if(database.getBootstrap()){
            // avoid optimisation
            exitRelease();
            continue;
        }

        // development feature - begin
        if(yamlConfig["debug"].IsDefined()){
            if(yamlConfig["debug"]["structureImageDump"].IsDefined()){
                for(auto viewpoint : database.viewpoints){
                    auto image = database.viewpointStructuralImage(viewpoint.get(), 0);
                    auto folder = yamlExport["path"].as<std::string>() + "/viewpointStructuresImages";
                    auto path = folder + "/" + std::to_string(viewpoint->index) + "_" + std::to_string(loopMajor) + "a.jpg";
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

        // Reset algorithm loop
        loopFlag=true;
        loopMinor=0;

        // Algorithm state loop
        while ( loopFlag == true ) {

            // algorithm optimisation loop
            while ( loopFlag == true ) {

                // Algorithm core
                database.computeModels(loopState);
                database.computeCentroids(loopState);
                database.computeCorrelations(loopState);
                database.computePoses(loopState);
                database.computeNormalisePoses(loopState);
                database.computeFrames(loopState);
                database.computeOriented(loopState);
                database.computeOptimals(loopState);
                database.computeRadii(loopState);

                // Stability filtering - radial limitation
                database.filterRadialRange(loopState);

                // Statistics computation on disparity
                database.computeDisparityStatistics(loopState);

                // Filtering on disparity
                database.filterDisparity(loopState);

                /* iteration end condition */
                loopFlag=database.getError(loopState, loopMajor, loopMinor);

                // Update minor iterator
                loopMinor ++;

            }

            // development feature - begin
            database._exportStructureModel(yamlExport["path"].as<std::string>(),loopMajor);
            // development feature - end

            // State loop management
            if((loopState==DB_MODE_BOOT)||(loopState==DB_MODE_FULL)){

                // Update loop mode
                loopState=DB_MODE_LAST;

            } else
            if (loopState==DB_MODE_LAST){

                // Update loop mode
                loopState=DB_MODE_FULL;

                // Continue optimisation
                loopFlag=true;

            }

        }

        // development feature - begin
        if(yamlConfig["debug"].IsDefined()){
            if(yamlConfig["debug"]["structureImageDump"].IsDefined()){
                for(auto viewpoint : database.viewpoints){
                    auto image = database.viewpointStructuralImage(viewpoint.get(), 0);
                    auto folder = yamlExport["path"].as<std::string>() + "/viewpointStructuresImages";
                    auto path = folder + "/" + std::to_string(viewpoint->index) + "_" + std::to_string(loopMajor) + "b.jpg";
                    fs::create_directories(folder);
                    cv::imwrite(path, image);
                }
            }
        }
        // development feature - end

        // Need to be placed after features extraction, as an image is not needed from there
        if ( database.viewpoints.size() > database.configMatchRange ) {

            // release viewpoint image memory
            database.viewpoints[database.viewpoints.size()-database.configMatchRange]->releaseImage();

        }

        // Major iteration exportation : model, odometry and transformation
        database.exportStructure     (yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor);
        database.exportPosition      (yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor);
        database.exportTransformation(yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor);

        // update major iterator
        loopMajor ++;

        exitRelease();

    }

    // Need release of : 
    //     ViewPointSource * source = NULL;
    //     Frontend * frontend(nullptr);

    // system message
    return 0;

}

