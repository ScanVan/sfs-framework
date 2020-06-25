#pragma once

#include "framework-database.hpp"
#include "framework-utiles.hpp"
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
	std::shared_ptr<Viewpoint> lastViewpoint;
	cv::Mat mask;
	Database *database;
	double scale;
public:

	void featureExtraction();
    FrontendPicture(ViewPointSource * source, cv::Mat mask, Database *database);
	virtual ~FrontendPicture(){}
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



