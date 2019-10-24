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

#pragma once


#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <stdint.h>
#include <vector>

#include "framework-viewpoint.hpp"

class ViewPointSource{
public:
	ViewPointSource() {}
	virtual ~ViewPointSource() {}
	virtual std::shared_ptr<Viewpoint> next() = 0;
	virtual bool hasNext() = 0;
};

class ViewPointSourceFs : public ViewPointSource{
private:
	std::vector<std::string> files;
	uint32_t fileIndex;
    double scale = 1.0;

public:
	ViewPointSourceFs(std::vector<std::string> files);
	ViewPointSourceFs(std::string folder, double scale);
	virtual ~ViewPointSourceFs() {}
	virtual std::shared_ptr<Viewpoint> next();
	virtual bool hasNext();
};
