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

//
//  File system and directories
//

void utilesDirectories(std::string rootPath, std::string modeName) {

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

}

//
//  Generic features
//

Eigen::Vector3d utilesDirection(double x, double y, int width, int height){

    // Convert pixel coordinates to geographic mapping coordinates
    double lamda((x/width)*2.*M_PI);
    double phi(((y/(height-1))-0.5)*M_PI);

    // Precompute cosine value
    double cosine(cos(phi));

    // Compute and return direction vector
    return Eigen::Vector3d(cosine*cos(lamda),cosine*sin(lamda),sin(phi));

}

//
//  Sparse features
//

void utilesAKAZEFeatures(cv::Mat* image, cv::Mat* mask, std::vector<cv::KeyPoint>* keypoints, cv::Mat* desc, float const threshold) {

    // Instance AKAZE feature detector
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, threshold, 4, 4, cv::KAZE::DIFF_PM_G2);

    // Compute feature and their descriptor
    akaze->detectAndCompute(*image, *mask, *keypoints, *desc);

}

void utilesGMSMatcher(std::vector<cv::KeyPoint>* k1, cv::Mat* d1, cv::Size s1, std::vector<cv::KeyPoint>* k2, cv::Mat* d2, cv::Size s2, std::vector<cv::DMatch> *matches) {

    // Raw matches
    std::vector<cv::DMatch> matches_all;

    // Instance brute-force matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);

    // GMS matcher inliers
	std::vector<bool> vbInliers;

    // Clear previous matches
    matches->clear();

    // Features matching
	matcher.match(*d1, *d2, matches_all);

    // Display matching summary
    std::cerr << "Feat 1 : " << k1->size() << " | Feat 2 : " << k2->size() << " | Match : " << matches_all.size();

	// GMS filtering on matches
	gms_matcher gms(*k1, s1, *k2, s2, matches_all);

    // Retrieve inliers flags
	gms.GetInlierMask(vbInliers, true, true);

    // Filter matches based on GMS filter
	for (size_t i = 0; i < vbInliers.size(); ++i) {

        // Detect inlier and push to the filtered match array
		if (vbInliers[i] == true) matches->push_back(matches_all[i]);

	}

    // Display matches filtering summary
    std::cerr << " | Filter : " << matches->size() << std::endl;

}

double utilesDetectMotion(std::vector<cv::KeyPoint> *kp1, std::vector<cv::KeyPoint> *kp2, std::vector<cv::DMatch> *matches, cv::Size size) {

	// Vector containing the distances for each pair of features
	std::vector<double> vec_dist {};

	// loop over the vector of matches
	for (const auto &m : *matches) {

		auto width = size.width;
		auto height = size.height;

		// m.queryIdx is the index of the Keypoints on the first image
		// m.trainIdx is the index of the Keypoints on the second image

		// Convert cartesian to spherical
		// Spherical coordinate of the feature on the first image
		Eigen::Vector3d  p1 = utilesDirection(static_cast<double>((*kp1)[m.queryIdx].pt.x), static_cast<double>((*kp1)[m.queryIdx].pt.y), width, height);
		// Spherical coordinate of the feature on the second image
		Eigen::Vector3d  p2 = utilesDirection(static_cast<double>((*kp2)[m.trainIdx].pt.x), static_cast<double>((*kp2)[m.trainIdx].pt.y), width, height);

		// p1*p2 computes the dot product between p1 and p2
		// push it to vector of distances
		vec_dist.push_back(acos(p1.dot(p2)));

	}

	// calculation of the mean value
	double m { std::accumulate(vec_dist.begin(), vec_dist.end(), 0.0) / vec_dist.size() };

	// calculation of variance
	double var { 0.0 };
	for (const auto & val : vec_dist) {
		var += pow(val - m, 2);
	}

	// check error to avoid division by 0
	if (vec_dist.size() <= 1) {
		throw(std::runtime_error("Number of features is less or equal to 1"));
	}
	var /= (vec_dist.size() - 1);

	// calculation of std
	double vec_std = sqrt(var);

	// return mean * std
	return m * vec_std;

}

//
//  Dense features
//

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

