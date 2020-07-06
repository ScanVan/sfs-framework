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

// External includes
#include "../lib/libflow/src/Cache.h"

// Internal includes
#include "framework-database.hpp"
#include "framework-utiles.hpp"
#include "framework-source.hpp"
#include "framework-utiles.hpp"

// Module object
class Frontend{

public:
	Frontend(){}
	virtual ~Frontend(){}
	virtual bool next() = 0;

};

// Module derived object
class FrontendPicture : public Frontend{

private:
	Source * source;
	std::shared_ptr<Viewpoint> lastViewpoint;
	cv::Mat mask;
	Database *database;
	double scale;
    float sparseThreshold;

public:
	void featureExtraction();
    FrontendPicture(Source * source, cv::Mat mask, Database *database, float const threshold);
	virtual ~FrontendPicture(){}
	virtual bool next();

};

// Module derived object
class FrontendDense : public Frontend{

private:
    Source * source;
    cv::Mat mask;
    Database *database;
    std::string ofCacheFolder;

public:
    FrontendDense(Source * source, cv::Mat mask, Database *database, std::string ofCacheFolder);
    virtual ~FrontendDense(){}
    virtual bool next();

};

