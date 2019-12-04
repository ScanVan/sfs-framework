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

#include "framework-utiles.hpp"

Eigen::Vector3d convertCartesian2Spherical(double x, double y, int width, int height){
	double lam( (x / width) * 2. * M_PI );
	double phi( ((y / (height-1)) - 0.5) * M_PI );
    Eigen::Vector3d pos3d( cos(phi)*cos(lam), cos(phi)*sin(lam), sin(phi) );
	return pos3d;
}

void profile(std::string msg){
	static struct timespec specOld;
	static std::string msgLast = "";
    struct timespec specNew;
    double t;

    clock_gettime(CLOCK_REALTIME, &specNew);

    if (specNew.tv_nsec >= 1000000000) {
    	specNew.tv_nsec -= 1000000000;
    	specNew.tv_sec++;
    }
    t = specNew.tv_sec - specOld.tv_sec + (specNew.tv_nsec - specOld.tv_nsec)*1e-9;
    std::cout << msgLast << " : " << t << std::endl;
    specOld = specNew;
    msgLast = msg;
}

std::mutex exitMutex;
int exitCounter = 0;

void exitRetain(){
    exitMutex.lock();
    exitCounter++;
    exitMutex.unlock();
}


void exitRelease(){
    exitMutex.lock();
    exitCounter--;
    if(exitCounter <= 0) exit(0);
    exitMutex.unlock();
}


double bilinear_sample(double *p, double x, double y, int width){
    int ix = x;
    int iy = y;

    int i00 = iy*width + ix;
    int i01 = i00 + 1;
    int i10 = i00 + width;
    int i11 = i00 + width + 1;

    double fx = x-ix;
    double fy = y-iy;

    return  (p[i00]*(1.0-fx) + p[i01]*fx)*(1.0-fy) + (p[i10]*(1.0-fx) + p[i11]*fx)*fy;
}
