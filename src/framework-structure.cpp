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

Eigen::Vector3d * Structure::getPosition(){
    return &position;
}

double Structure::getError(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    double error(0.);
    double candidate(0.);
    for(unsigned int i(0); i<vplink.size(); i++){
        candidate=viewpoints[vplink[i]]->getDisparity(ftlink[i])>error;
        if (candidate>error) error=candidate;
    }
    return error;
}

void Structure::computeCorrelation(std::vector<std::shared_ptr<Viewpoint>> & viewpoints, std::vector<std::shared_ptr<Transform>> & transforms){
    for(unsigned int i(0); i<vplink.size(); i++){
        for(unsigned int j(0); j<vplink.size(); j++){
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

void Structure::computeOptimalPosition(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    Eigen::Matrix3d macc(Eigen::Matrix3d::Zero());
    Eigen::Vector3d vacc(Eigen::Vector3d::Zero());
    for(unsigned int i(0); i<vplink.size(); i++){
        Eigen::Vector3d *dirvec(viewpoints[vplink[i]]->getDirection(ftlink[i]));
        Eigen::Matrix3d weight(Eigen::Matrix3d::Identity()-(*dirvec)*(*dirvec).transpose());
        macc+=weight;
        vacc+=weight*(*viewpoints[vplink[i]]->getPosition());
    }
    position=macc.inverse()*vacc;
}

void Structure::computeRadius(std::vector<std::shared_ptr<Viewpoint>> & viewpoints){
    for(unsigned int i(0); i<vplink.size(); i++){
        Eigen::Vector3d fdirection(*viewpoints[vplink[i]]->getDirection(ftlink[i]));
        Eigen::Matrix3d * vorientation(viewpoints[vplink[i]]->getOrientation());
        Eigen::Vector3d * vposition(viewpoints[vplink[i]]->getPosition());
        fdirection=(*vorientation)*fdirection;
        double radius(fdirection.dot(position-(*vposition)));
        viewpoints[vplink[i]]->setRadius(ftlink[i], radius, ((*vposition)+fdirection*radius-position).norm());
    }
}
