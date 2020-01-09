#include "Opencv.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <experimental/filesystem>
#include "Image.h"
#include "OpticalFlow.h"

namespace fs = std::experimental::filesystem;

cv::Mat sv_dense_io_image( char const * const sv_path, double const sv_scale ) {

    /* matrix variable */
    cv::Mat sv_import;

    /* matrix variable */
    cv::Mat sv_image;

    /* import and check image */
    if ( ! ( sv_import = cv::imread( sv_path, cv::IMREAD_COLOR ) ).data ) {

        /* display message */
        std::cerr << "scanvan : error : unable to import image" << std::endl;

        /* send message */
        exit( 1 );

    }

    /* resize image */
    cv::resize( sv_import, sv_image, cv::Size(), sv_scale, sv_scale, cv::INTER_AREA );

    /* convert image to double */
    sv_image.convertTo( sv_image, CV_64FC3 );

    /* image renormalisation */
    sv_image /= 255.0;

    /* return read image */
    return( sv_image );

}

int sv_dense_flow( cv::Mat & sv_img_a, cv::Mat & sv_img_b, long const sv_width, long const sv_height, long const sv_depth, DImage & sv_flow_u, DImage & sv_flow_v ) {

    /* image length variable */
    long sv_length( sv_width * sv_height * sv_depth );

    /* image variable */
    DImage sv_dimg_a;
    DImage sv_dimg_b;

    /* image variable */
    DImage sv_warp;

    /* allocate image memory */
    sv_dimg_a.allocate( sv_width, sv_height, sv_depth );

    /* assign color type */
    sv_dimg_a.setColorType( 0 );

    /* allocate image memory */
    sv_dimg_b.allocate( sv_width, sv_height, sv_depth );

    /* assign color type */
    sv_dimg_b.setColorType( 0 );

    /* copy image content */
    memcpy( sv_dimg_a.pData, sv_img_a.data, sv_length * sizeof( double ) );

    /* copy image content */
    memcpy( sv_dimg_b.pData, sv_img_b.data, sv_length * sizeof( double ) );

    /* compute optical flow */
    OpticalFlow::Coarse2FineFlow( sv_flow_u, sv_flow_v, sv_warp, sv_dimg_a, sv_dimg_b, 0.012, 0.75, 20, 7, 1, 30 );

}

void writeUv(DImage & fu, DImage & fv, cv::Mat & mat){
    mat.create(fu.height(),fu.width(),CV_8UC3);
    auto u = &fu[0];
    auto v = &fv[0];
    for(int y = 0;y < fu.height();y++){
        for(int x = 0;x < fu.width();x++){
            mat.at<cv::Vec3b>(y,x)[0] = atan2(*u, *v)/M_PI/2*256;
            mat.at<cv::Vec3b>(y,x)[1] = MIN(255, sqrt(*u**u +*v**v)*10);
            mat.at<cv::Vec3b>(y,x)[2] =  255;

            u++;
            v++;
        }
    }

    cv::cvtColor(mat, mat, cv::COLOR_HSV2BGR);
}

void writeUv(cv::Mat &u, cv::Mat &v, cv::Mat & mat){
    mat.create(u.rows, u.cols, CV_8UC3);
    for(int y = 0;y < u.rows;y++){
        for(int x = 0;x < u.cols;x++){
            auto eu = u.at<float>(y,x);
            auto ev = v.at<float>(y,x);
            mat.at<cv::Vec3b>(y,x)[0] = atan2(eu, ev)/M_PI/2*256;
            mat.at<cv::Vec3b>(y,x)[1] = MIN(255, sqrt(eu*eu + ev*ev)*10);
            mat.at<cv::Vec3b>(y,x)[2] =  255;
        }
    }

    cv::cvtColor(mat, mat, cv::COLOR_HSV2BGR);
}

