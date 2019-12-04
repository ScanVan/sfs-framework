///*
// *  sfs-framework
// *
// *      Nils Hamel - nils.hamel@bluewin.ch
// *      Charles Papon - charles.papon.90@gmail.com
// *      Copyright (c) 2019 DHLAB, EPFL & HES-SO Valais-Wallis
// *
// *  This program is free software: you can redistribute it and/or modify
// *  it under the terms of the GNU General Public License as published by
// *  the Free Software Foundation, either version 3 of the License, or
// *  (at your option) any later version.
// *
// *  This program is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// *  GNU General Public License for more details.
// *
// *  You should have received a copy of the GNU General Public License
// *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */
//
//#include "framework.hpp"
//
//
//#include "framework.hpp"
//#include <experimental/filesystem>
//
//namespace fs = std::experimental::filesystem;
//
//
//int main(int argc, char *argv[]){
//    std::cout << "Hello dense world!" << std::endl;
//
//
//    assert(argc == 2);
//    std::cout << "Hello world!" << std::endl;
//
//    YAML::Node config = YAML::LoadFile(argv[1]);
//    auto database = Database(
//        config["algorithm"]["error"].as<double>(),
//        config["algorithm"]["disparity"].as<double>(),
//        config["algorithm"]["radius"].as<double>()
//    );
//
//    Frontend *frontend = NULL;
//    auto frontendType = config["frontend"]["type"].as<std::string>();
//    if(frontendType == "IMAGE"){
//        auto fs = config["frontend"]["source"];
//        ViewPointSource *source = NULL;
//        auto sourceType = fs["type"].as<std::string>();
//        if(sourceType == "FOLDER") {
//            std::string firstFile =  fs["first"].IsDefined() ? fs["first"].as<std::string>() : "";
//            std::string lastFile = fs["last"].IsDefined() ? fs["last"].as<std::string>() : "";
//            source = new ViewPointSourceFs(fs["path"].as<std::string>(), fs["scale"].as<double>(), firstFile, lastFile);
//        }
//        auto mask = cv::imread(fs["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
//        if(fs["scale"].IsDefined()){
//            auto scale = fs["scale"].as<double>();
//            cv::resize(mask, mask, cv::Size(), scale, scale, cv::INTER_AREA );
//        }
//        frontend = new FrontendPicture(source, mask, &threadpool, &database);
//    }
//
//    return 0;
//}
//
