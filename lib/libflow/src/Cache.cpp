#include "Cache.h"

#include <unordered_map>
#include <experimental/filesystem>
#include <opencv2/img_hash.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iomanip>

#include "Image.h"
#include "OpticalFlow.h"
#include "Opencv.h"

namespace fs = std::experimental::filesystem;

void matwrite(const string& filename, const cv::Mat& mat)
{
    ofstream fs(filename, fstream::binary);

    // Header
    int type = mat.type();
    int channels = mat.channels();
    fs.write((char*)&mat.rows, sizeof(int));    // rows
    fs.write((char*)&mat.cols, sizeof(int));    // cols
    fs.write((char*)&type, sizeof(int));        // type
    fs.write((char*)&channels, sizeof(int));    // channels

    // Data
    if (mat.isContinuous())
    {
        fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
    }
    else
    {
        int rowsz = CV_ELEM_SIZE(type) * mat.cols;
        for (int r = 0; r < mat.rows; ++r)
        {
            fs.write(mat.ptr<char>(r), rowsz);
        }
    }
}

cv::Mat matread(const string& filename)
{
    ifstream fs(filename, fstream::binary);

    // Header
    int rows, cols, type, channels;
    fs.read((char*)&rows, sizeof(int));         // rows
    fs.read((char*)&cols, sizeof(int));         // cols
    fs.read((char*)&type, sizeof(int));         // type
    fs.read((char*)&channels, sizeof(int));     // channels

    // Data
    cv::Mat mat(rows, cols, type);
    fs.read((char*)mat.data, CV_ELEM_SIZE(type) * rows * cols);

    return mat;
}

void ofCache(cv::Mat &a, cv::Mat &b, cv::Mat &u, cv::Mat &v, std::string cache){
    auto keyInt = std::_Hash_impl::hash(a.data, a.dataend - a .datastart) ^ (std::_Hash_impl::hash(b.data, b.dataend - b .datastart) << 1);
    std::stringstream sstream;
    sstream << std::hex << std::setfill('0') << std::setw(16) << keyInt;
    std::string keyString = sstream.str();
    auto filePath = cache + "/" + keyString;
    if(!std::ifstream((filePath + ".png").c_str()).good()){
        DImage du;
        DImage dv;
        sv_dense_flow(a, b, a.cols, a.rows, a.channels(), du, dv);

        u = cv::Mat(a.rows, a.cols, CV_32FC1);
        v = cv::Mat(a.rows, a.cols, CV_32FC1);
        for(int y = 0;y < a.rows; y++){
            for(int x = 0;x < a.cols; x++){
                auto i = y*a.cols+x;
                u.at<float>(y,x) = du.data()[i];
                v.at<float>(y,x) = dv.data()[i];
            }
        }

        matwrite(filePath + "_u", u);
        matwrite(filePath + "_v", v);

        cv::Mat humanMat;
        writeUv(u, v, humanMat);
        cv::imwrite(filePath + ".png", humanMat);
    } else {
        u = matread(filePath + "_u");
        v = matread(filePath + "_v");
    }
}
