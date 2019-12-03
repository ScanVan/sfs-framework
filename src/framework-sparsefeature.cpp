

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>




void akazeFeatures(	cv::Mat* image,
					cv::Mat* mask,
					std::vector<cv::KeyPoint>* keypoints,
					cv::Mat* desc) {

	// The following parameters worked for the sequence 1-34 of the Sion Outdoor dataset
	cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f, 4, 4, cv::KAZE::DIFF_PM_G2);
    //cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.00025f, 4, 4, cv::KAZE::DIFF_PM_G2);

	akaze->detectAndCompute(*image, *mask, *keypoints, *desc);
}

#include "gms_matcher.h"
void gmsMatcher (	std::vector<cv::KeyPoint>* k1,
					cv::Mat* d1,
					cv::Size s1,
					std::vector<cv::KeyPoint>* k2,
					cv::Mat* d2,
					cv::Size s2,
					std::vector<cv::DMatch> *matches) {
    std::vector<cv::DMatch> matches_all;
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
//    cv::FlannBasedMatcher matcher;
//    if(d1->type()!=CV_32F) {
//        d1->convertTo(*d1, CV_32F);
//    }
//
//    if(d2->type()!=CV_32F) {
//        d2->convertTo(*d2, CV_32F);
//    }

    // Second param is boolean variable, crossCheck which is false by default. If it is true, Matcher returns only those matches with value (i,j) such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa. That is, the two features in both sets should match each other. It provides consistent result, and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
	matcher.match(*d1, *d2, matches_all);

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
}



