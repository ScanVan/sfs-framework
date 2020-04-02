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

#pragma once

// External include
#include <stdint.h>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <experimental/filesystem>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>

// Internal includes
#include "framework-viewpoint.hpp"

// Namespaces
namespace fs = std::experimental::filesystem;

// Module object
class ViewPointSource{
protected:
    int fileIndex;
    int fileLastIndex;
    int fileIncrement;
    double imageScale;
public:
	ViewPointSource() {}
    ViewPointSource(int increment, double scale) : fileIndex(-1), fileLastIndex(-1), fileIncrement(increment), imageScale(scale) {}
	virtual ~ViewPointSource() {}
	virtual std::shared_ptr<Viewpoint> next() = 0;
	virtual bool hasNext() = 0;
};

// Module derived object
class ViewPointSourceFs : public ViewPointSource{
private:
	std::vector<std::string> files;
public:
    ViewPointSourceFs(std::string imageFolder, std::string firstImage, std::string lastImage, uint32_t increment, double scale);
	~ViewPointSourceFs() {}
	std::shared_ptr<Viewpoint> next();
	bool hasNext();
};

// Module derived object
class ViewPointSourceWithOdometry : public ViewPointSource{
private:
    class ViewPointInfo{
    public:
        std::string fileName;
        Eigen::Vector3d position;
        Eigen::Matrix3d orientation;
    };
    std::vector<ViewPointInfo> list;
    std::string pictureFolder;
public:
    ViewPointSourceWithOdometry(std::string imageFolder, std::string transformationFile, std::string firstFile, std::string lastFile, int increment, double scale);
    ~ViewPointSourceWithOdometry() {}
    std::shared_ptr<Viewpoint> next();
    bool hasNext();
};

