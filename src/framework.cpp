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

//
//  Main function - Framework front-end
//

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
        std::cerr << "Missing section in YAML configuration file (frontend)" << std::endl;
        return 1;
    } else
    if ( yamlConfig["matching"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file (matching)" << std::endl;
        return 1;
    } else
    if ( yamlConfig["algorithm"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file (algorithm)" << std::endl;
        return 1;
    } else
    if ( yamlConfig["export"].IsDefined() != true ) {
        // Display error and exit
        std::cerr << "Missing section in YAML configuration file (export)" << std::endl;
        return 1;
    }

    // Instance of main yaml section
    YAML::Node yamlFrontend  = yamlConfig["frontend"];
    YAML::Node yamlFeatures  = yamlConfig["features"];
    YAML::Node yamlMatching  = yamlConfig["matching"];
    YAML::Node yamlAlgorithm = yamlConfig["algorithm"];
    YAML::Node yamlDense     = yamlConfig["dense"];
    YAML::Node yamlExport    = yamlConfig["export"];

    //
    //  Framework initialisation
    //

    // Framework main structure initialisation
    Database database(
        yamlAlgorithm["error"].as<double>(),
        yamlAlgorithm["disparity"].as<double>(),
        yamlAlgorithm["radius"].as<double>(),
        yamlAlgorithm["group"].as<unsigned int>(),
        yamlMatching["range"].as<unsigned int>(),
        yamlDense["disparity"].as<double>()
    );

    // Framework front-end
    Frontend * frontend(nullptr);

    // Framework data source
    Source * source(nullptr);

    // Pipeline iterations
    int loopMajor(database.getGroup());
    int loopMinor(0);

    // Loop flag
    bool pipeFlag(true);
    bool loopFlag(true);

    // Algorithm state
    int loopState(DB_MODE_NULL);

    //
    //  Framework exportation
    //

    // Create exportation directories
    utilesDirectories(yamlExport["path"].as<std::string>(), yamlFrontend["type"].as<std::string>());

    //
    //  Framework front-end
    //

    // Switch on front-end : odometry (sparse), densification (dense)
    if(yamlFrontend["type"].as<std::string>() == "sparse"){

        // Detect image list boundary
        std::string firstFile = yamlFrontend["first"].IsDefined() ? yamlFrontend["first"].as<std::string>() : "";
        std::string lastFile  = yamlFrontend["last" ].IsDefined() ? yamlFrontend["last" ].as<std::string>() : "";

        // Front-end source
        source = new SourceSparse(
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
        frontend = new FrontendPicture(source, mask, &database, yamlFeatures["threshold"].as<float>());

        // Initialise algorithm state
        loopState = DB_MODE_BOOT;

    } else
    if(yamlFrontend["type"].as<std::string>() == "dense"){

        // Detect image list boundary
        std::string firstFile = yamlFrontend["first"].IsDefined() ? yamlFrontend["first"].as<std::string>() : "";
        std::string lastFile  = yamlFrontend["last" ].IsDefined() ? yamlFrontend["last" ].as<std::string>() : "";

        // Front-end source
        source = new SourceDense(
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
        frontend = new FrontendDense(source, mask, &database, yamlExport["path"].as<std::string>() + "/cache" );

        // Initialise algorithm state
        loopState = DB_MODE_MASS;

    }

    //
    //  Framework optimisation algorithm
    //

    // Framework main loop
    while( pipeFlag == true ){

        // Query image from source
        if(frontend->next()==false){
            pipeFlag=false;
            if(loopState==DB_MODE_MASS){
                break;
            }else{
                loopState=DB_MODE_FULL;
            }
        }

        // Start optimisation only if sufficient amount of view are pushed
        if(database.getBootstrap()){
            continue;
        }

        // Update pipeline state
        database.prepareState(loopState);

        // Prepare structures
        database.prepareStructures();

        // Reset loop flag
        loopFlag=true;

        // Reset iteration
        loopMinor=0;

        // Algorithm loop
        while ( loopFlag == true ) {

            // Push scale information
            database.prepareTransforms();

            // Optimisation loop
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

                /* Iteration end condition */
                loopFlag=database.getError(loopState, loopMajor, loopMinor);

                // Update minor iterator
                loopMinor ++;

            }

            // State loop management
            if(loopState==DB_MODE_LAST){

                // Broacast scale information
                database.broadcastScale();

                // Update mode for algorithm pseudo-iteration
                loopState = database.prepareState(DB_MODE_FULL);

                // Algorithm pseudo-core
                database.computeFrames(loopState);
                database.computeOriented(loopState);
                database.computeOptimals(loopState);
                database.computeRadii(loopState);

                // Stability filtering - radial limitation
                database.filterRadialRange(loopState);

                // Continue optimisation
                //loopFlag=true;
                loopState=DB_MODE_LAST;

            }else if(loopState==DB_MODE_BOOT){

                // Update loop mode
                loopState=DB_MODE_LAST;

            }else if(loopState==DB_MODE_FULL){

                // Update loop mode
                loopState=DB_MODE_LAST;

            }

        }

        // Expunge filtered structures
        database.expungeStructures();

        // Major iteration exportation : model, odometry, transformation and constraint
        database.exportStructure     (yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor,yamlExport["group"].as<unsigned int>());
        database.exportPosition      (yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor);
        database.exportTransformation(yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor);
        database.exportConstraint    (yamlExport["path"].as<std::string>(),yamlFrontend["type"].as<std::string>(),loopMajor,yamlExport["group"].as<unsigned int>());

        // update major iterator
        loopMajor ++;

    }

    // Delete frontend object
    delete frontend;

    // Delete source object
    delete source;

    // system message
    return 0;

}

