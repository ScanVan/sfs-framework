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

# include "framework-structure.hpp"

void Structure::computeCorrelation(std::vector<std::shared_ptr<Viewpoint>> & viewpoints, std::vector<std::shared_ptr<Transform>> & transforms){
    for(int i(0); i<vplink.size(); i++){
        for(int j(0); j<vplink.size(); j++){
            if (vplink[i]-vplink[j] == 1){
                int vpb(vplink[i]), vpa(vplink[j]);
                int ftb(ftlink[i]), fta(ftlink[j]);
                transforms[vpb]->pushCorrelation(
                    viewpoints[vpa]->getModelPoint(fta), viewpoints[vpa]->getCentroid(),
                    viewpoints[vpb]->getModelPoint(ftb), viewpoints[vpb]->getCentroid()
                );
            }
        }
    }
}
