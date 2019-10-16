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
public:

	void featureExtraction();
	FrontendPicture(ViewPointSource * source, cv::Mat mask, ThreadPool *threadpool, Database *database);
	virtual ~FrontendPicture(){}

	virtual bool next();
};


class FrontendCloudpoint : public Frontend{
private:
	std::shared_ptr<Viewpoint> lastViewpoint;
	Database *database;
	std::vector<Eigen::Vector3d> model, odometry;
public:
	FrontendCloudpoint(Database *database, std::string modelPath, std::string odometryPath);
	virtual ~FrontendCloudpoint(){}

	virtual bool next();
};
