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

#include <string>
#include <Eigen/Dense>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>

class viewpoint {

    private:

        std::string                     uid;        /* name of the view point - ex : 20181010-144724-156648 */
        cv::Mat                         image;      /* image pixel data - maybe not needed */
        std::vector< KeyPoint >         features;   /* features 2D */
        cv::OutputArray                 descriptor; /* features descriptors */
        std::vector< Eigen::Vector3d >  direction;  /* features direction in the viewpoint frame - unit vectors */
        std::vector< double >           radius;     /* features radius according to the corresponding direction */
        Eigen::Vector3d                 position;   /* position of the viewpoint in the common frame */

};
