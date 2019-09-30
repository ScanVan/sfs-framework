#include "framework-stillcompute.hpp"

#include <cmath>
#include <Eigen/Core>
#include "framework-utiles.hpp"
#include <numeric>

double computeStillDistance(std::vector<cv::KeyPoint> *kp1,
							std::vector<cv::KeyPoint> *kp2,
							std::vector<cv::DMatch> *matches,
							cv::Size size) {
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
		Eigen::Vector3d  p1 = convertCartesian2Spherical(static_cast<double>((*kp1)[m.queryIdx].pt.x), static_cast<double>((*kp1)[m.queryIdx].pt.y), width, height);
		// Spherical coordinate of the feature on the second image
		Eigen::Vector3d  p2 = convertCartesian2Spherical(static_cast<double>((*kp2)[m.trainIdx].pt.x), static_cast<double>((*kp2)[m.trainIdx].pt.y), width, height);

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
