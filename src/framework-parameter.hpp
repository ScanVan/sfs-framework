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

#include <cmath>

class Parameter {
private:
    double iteration_error;  /* stop condition on geometry estimation : 1e-8 */
    double disparity;
    double triangulation;
public:
    Parameter(void) : iteration_error( 1e-8 ), disparity( ( ( 2.0 * M_PI ) / 6016 ) * 0.2 ), triangulation( ( M_PI / 180.0 ) * 1.5 ) {}
    double getError(){
        return iteration_error;
    }
    double getDisparity(){
        return disparity;
    }
    double getTriangulation(){
        return triangulation;
    }
};
