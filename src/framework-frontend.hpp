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
