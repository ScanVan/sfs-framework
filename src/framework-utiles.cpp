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


float bilinear_sample(float *p, float x, float y, int width){
    int ix = x;
    int iy = y;

    int i00 = iy*width + ix;
    int i01 = i00 + 1;
    int i10 = i00 + width;
    int i11 = i00 + width + 1;

    float fx = x-ix;
    float fy = y-iy;

    return  (p[i00]*(1.0f-fx) + p[i01]*fx)*(1.0f-fy) + (p[i10]*(1.0f-fx) + p[i11]*fx)*fy;
}

void create_directories( std::string rootPath, std::string modeName ) {

    // Sub-sequent path variable
    std::string modePath( rootPath + "/" + modeName );

    // Create directories
    fs::create_directories( rootPath.c_str() );
    fs::create_directory  ( modePath.c_str() );

    // Specific directory
    if (modeName == "dense" ){

        // Create path
        modePath = rootPath + "/cache";

        // Create directory
        fs::create_directory( modePath.c_str() );

    }

    // Create debug
    fs::create_directory( rootPath + "/debug" );

}

Eigen::Vector3d compute_intersection(Eigen::Vector3d * p1, Eigen::Vector3d * d1, Eigen::Vector3d * p2, Eigen::Vector3d * d2){

    Eigen::Vector3d d0( *p2 - *p1 );

    double aa(d1->dot(*d1));
    double bb(d2->dot(*d2));
    double ab(d1->dot(*d2));

    double ac(d1->dot(d0));
    double bc(d2->dot(d0));

    double dn(aa*bb-ab*ab);

    double r1((- ab * bc + ac * bb )/dn);
    double r2((+ ab * ac - bc * aa )/dn);

    d0 = 0.5 * ( *p1+(*d1*r1) + *p2+(*d2*r2) );

    return d0;
}

//function [ d_1_r, d_2_r, d_inter ] = duplet_intersect( d_1_p, d_1_d, d_2_p, d_2_d )

//    % intermediate computation %
//    d_0_d = d_2_p - d_1_p;

//    % intermediate computation %
//    d_aa = dot( d_1_d, d_1_d );
//    d_bb = dot( d_2_d, d_2_d );
//    d_ab = dot( d_1_d, d_2_d );
//    d_ac = dot( d_1_d, d_0_d );
//    d_bc = dot( d_2_d, d_0_d );

//    % intermediate computation %
//    d_dn = d_aa * d_bb - d_ab * d_ab;

//    % compute radius %
//    d_1_r = ( - d_ab * d_bc + d_ac * d_bb ) / d_dn;
//    d_2_r = ( + d_ab * d_ac - d_bc * d_aa ) / d_dn;

//    % compute intersection %
//    d_inter = 0.5 * ( d_1_p + d_1_d * d_1_r + d_2_p + d_2_d * d_2_r );

//end

