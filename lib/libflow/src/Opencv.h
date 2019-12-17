#pragma once

#include <opencv2/core.hpp>
#include "Image.h"
#include "OpticalFlow.h"


cv::Mat sv_dense_io_image( char const * const sv_path, double const sv_scale );
int sv_dense_flow( cv::Mat & sv_img_a, cv::Mat & sv_img_b, long const sv_width, long const sv_height, long const sv_depth, DImage & sv_flow_u, DImage & sv_flow_v );
void writeUv(DImage & fu, DImage & fv, cv::Mat & mat);
void writeUv(cv::Mat &u, cv::Mat &v, cv::Mat & mat);
