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

#include "framework-sparsefeature.hpp"

void akazeFeatures(cv::Mat* image, cv::Mat* mask, std::vector<cv::KeyPoint>* keypoints, cv::Mat* desc, float const threshold) {

    // Instance AKAZE feature detector
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, threshold, 4, 4, cv::KAZE::DIFF_PM_G2);

    // Compute feature and their descriptor
    akaze->detectAndCompute(*image, *mask, *keypoints, *desc);

}

void gmsMatcher (	std::vector<cv::KeyPoint>* k1,
					cv::Mat* d1,
					cv::Size s1,
					std::vector<cv::KeyPoint>* k2,
					cv::Mat* d2,
					cv::Size s2,
					std::vector<cv::DMatch> *matches) {
    std::vector<cv::DMatch> matches_all;
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    //cv::FlannBasedMatcher matcher;
    //if(d1->type()!=CV_32F) {
    //    d1->convertTo(*d1, CV_32F);
    //}
    //if(d2->type()!=CV_32F) {
    //    d2->convertTo(*d2, CV_32F);
    //}

    // Second param is boolean variable, crossCheck which is false by default. If it is true, Matcher returns only those matches with value (i,j) such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa. That is, the two features in both sets should match each other. It provides consistent result, and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
	matcher.match(*d1, *d2, matches_all);

    std::cerr << "Feat 1 : " << k1->size() << " | Feat 2 : " << k2->size() << " | Match : " << matches_all.size();

	// GMS filter
	std::vector<bool> vbInliers;
	gms_matcher gms(*k1, s1, *k2, s2, matches_all);
	gms.GetInlierMask(vbInliers, true, true);

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		// if inliner, then push into valid matches
		if (vbInliers[i] == true)
		{
			matches->push_back(matches_all[i]);
		}
	}

    std::cerr << " | Filter : " << matches->size() << std::endl;
}



