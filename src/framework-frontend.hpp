#pragma once

#include "framework-database.hpp"
#include "framework-utiles.hpp"
#include "ThreadPool.h"
#include "framework-viewpointsource.hpp"


class Frontend{
public:
	Frontend(){}
	virtual ~Frontend(){}

	virtual bool next() = 0;
};

class FrontendPicture : public Frontend{
private:
	ViewPointSource * source;
	ThreadPool *threadpool;
	std::shared_ptr<Viewpoint> lastViewpoint;
	cv::Mat mask;
	BlockingQueue<std::shared_ptr<Viewpoint>> featureExtractionQueue;
	Database *database;
	std::thread featureExtractionThread;
	double scale;
public:

	void featureExtraction();
	FrontendPicture(ViewPointSource * source, cv::Mat mask, ThreadPool *threadpool, Database *database);
	virtual ~FrontendPicture(){}

	virtual bool next();
};


class FrontendCloudpoint : public Frontend{
private:
	Database *database;
	std::vector<Eigen::Vector3d> model, odometry;
	uint32_t viewpointIndex = 0;
	double distanceMax, badMatchRate, baseNoise, badMatchNoise;
public:
	FrontendCloudpoint(Database *database, std::string modelPath, std::string odometryPath, double distanceMax, double badMatchRate, double baseNoise, double badMatchNoise);
	virtual ~FrontendCloudpoint(){}

	virtual bool next();
};

class FrontendDense : public Frontend{
private:
    ViewPointSource * source;
    cv::Mat mask;
    Database *database;
    std::string ofCacheFolder;

public:
    FrontendDense(ViewPointSource * source, cv::Mat mask, Database *database, std::string ofCacheFolder);
    virtual ~FrontendDense(){}
    virtual bool next();
};



