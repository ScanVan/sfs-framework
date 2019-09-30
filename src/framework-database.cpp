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

#include "framework-database.hpp"

void Database::computeModels(){
    for(unsigned int i(0); i<viewpoints.size(); i++){
        viewpoints[i]->computeModel();
        viewpoints[i]->computeCentroid();
    }
}

void Database::computeCorrelations(){
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->resetCorrelation();
    }
    for(unsigned int i(0); i<structures.size(); i++){
        structures[i]->computeCorrelation(viewpoints,transforms);
    }
}

void Database::computePoses(){
    for(unsigned int i(1); i<transforms.size(); i++){
        transforms[i]->computePose(viewpoints[i-1].get(),viewpoints[i].get());
    }
}

void Database::computeFrame(){
    viewpoints[0]->resetFrame();
    for(unsigned int i(0); i<transforms.size(); i++){
        transforms[i]->computeFrame(viewpoints[i].get(),viewpoints[i+1].get());
    }
}
