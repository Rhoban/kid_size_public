#include "Filters/Basics/Undistort.hpp"

#define FRAME_640_480

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "rhoban_utils/timing/benchmark.h"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include "opencv2/ocl/ocl.hpp"

using rhoban_utils::Benchmark;

namespace Vision {
namespace Filters {

using namespace cv;
using namespace std;

class MyData {
public:
  MyData() {}

  void write(FileStorage &fs) const // Write serialization for this class
  {
    std::cout << "Write not implemented" << std::endl;
  }
  void read(const FileNode &node) // Read serialization for this class
  {
    /*    A = (int)node["A"];
          X = (double)node["X"];
          id = (string)node["id"];*/
  }

public:
  Mat mat1;
  Mat mat2;
};

Undistort::Undistort() : Filter("Undistort") { _first = true; }

cv::Point Undistort::predecessor(const cv::Point &p) const {
  cv::Point result;
  result.x = _map1Inverted.at<cv::Vec2s>(p)[0];
  result.y = _map1Inverted.at<cv::Vec2s>(p)[1];
  return result;
}

void Undistort::setParameters() {}

cv::Mat Undistort::undistortOneShot(cv::Mat input) {
  cv::Mat output;
  cv::remap(input, output, _map1Inverted, _map2, cv::INTER_LINEAR);
  return output;
}

void Undistort::process() {
  Benchmark::open("input");
  cv::Mat input = *(getDependency().getImg());
  Benchmark::close("input");

  if (_first) {
    _first = false;
    Benchmark::open("initUndistortRectifyMap (should be done only once)");
    // calculating the correction matrix only once
    // float arrayCameraMatrix[9] = {4.3139353418002967e+04, 0.0,
    // 6.2337587562428484e+02, 0.0, 4.3139353418002967e+04,
    // 4.1767128535606531e+02, 0.0, 0.0, 1.0};

    // float arrayCameraMatrix[9] =  {7.9947937632348817e+02, 0.0,
    // 6.5455245262936342e+02, 0.0, 7.9914212207933542e+02,
    // 4.7008140525345215e+02, 0.0, 0.0, 1.0};
    // cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, arrayCameraMatrix);
    // float arrayDistCoeffs[5] = {-2.3236841793965027e+02,
    // -1.7423332497272844e+05, 7.9223591037629315e-01, 1.4007423113959502e-01,
    // -6.3804952758025053e+01};
    // float arrayDistCoeffs[5] = {-6.3670467542523299e-02, 0.0, 0.0, 0.0};
    // cv::Mat distCoeffs= cv::Mat(5, 1, CV_32F, arrayDistCoeffs);
    cv::Size imageSize;

    imageSize = input.size();

    // cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
    //                             getOptimalNewCameraMatrix(cameraMatrix,
    //                             distCoeffs, imageSize, 1, imageSize, 0),
    //                             imageSize, CV_16SC2, _map1, _map2);
    Benchmark::close("initUndistortRectifyMap (should be done only once)");
    Benchmark::open("Reading huge map1 and map2 (only once)");
    cout << endl
         << "Reading big_mama.xml ..." << endl;
    FileStorage fs;
    std::string filename = "big_mama.xml"; // FIXME
    // MAIS DE QUOI FIXME? Il est très bien ce nom.
    fs.open(filename, FileStorage::READ);

    if (!fs.isOpened()) {
      throw runtime_error("Failed to open " + filename);
    }

    fs["map1"] >> _map1;
#ifdef FRAME_640_480
    _map1Inverted = cv::Mat(640, 480, CV_16SC2);
#else
    _map1Inverted = cv::Mat(1280, 960, CV_16SC2);
#endif

// map2 is useless since map1 is x,y
// If inverted use :
#ifdef FRAME_640_480
    _map2 = cv::Mat(640, 480, CV_16UC1);
#else
    _map2 = cv::Mat(1280, 960, CV_16UC1);
#endif

    // If normal use :
    //	_map2 = cv::Mat(960, 1280, CV_16UC1);
    //	fs["map2"] >> _map2;
    // std::cout << "map1 type = " << _map1.type() << std::endl;
    // std::cout << "map2 type = " << _map2.type() << std::endl;
    /*
      C1  C2  C3  C4
      CV_8U   0   8   16  24
      CV_8S   1   9   17  25
      CV_16U  2   10  18  26
      CV_16S  3   11  19  27
      CV_32S  4   12  20  28
      CV_32F  5   13  21  29
      CV_64F  6   14  22  30
    */

    // FIXME: Transpose. De quoi FIXME? FIXME toi même.
    for (int i = 0; i < _map1Inverted.rows; i++) {
      for (int j = 0; j < _map1Inverted.cols; j++) {
        // short temp = _map1.at<cv::Vec2s>(i,j)[0];
        // Works for first log (was the camera upside down?)
        //	_map1Inverted.at<cv::Vec2s>(i,j)[0] =
        //_map1.at<cv::Vec2s>(j,i)[0];
        //	_map1Inverted.at<cv::Vec2s>(i,j)[1] =
        //_map1.at<cv::Vec2s>(j,i)[1];
        // #ifdef FRAME_640_480
        //               _map1Inverted.at<cv::Vec2s>(i,j)[0] =
        //               _map1.at<cv::Vec2s>(2*j,_map1.cols - 2*i - 1)[0]/2;
        //               _map1Inverted.at<cv::Vec2s>(i,j)[1] =
        //               _map1.at<cv::Vec2s>(2*j,_map1.cols - 2*i - 1)[1]/2;
        // #else
        _map1Inverted.at<cv::Vec2s>(i, j)[0] =
            _map1.at<cv::Vec2s>(j, _map1Inverted.rows - i - 1)[0];
        _map1Inverted.at<cv::Vec2s>(i, j)[1] =
            _map1.at<cv::Vec2s>(j, _map1Inverted.rows - i - 1)[1];
        // #endif
      }
    }

    /*
      for(int i=0; i<_map2.rows; i++)
      for(int j=0; j<_map2.cols; j++)
      _map2.at<short>(i,j) = _map2.at<short>(i,j)/8;
    */
    Benchmark::close("Reading huge map1 and map2 (only once)");

    if (Filter::GPU_ON) {
      map1.upload(_map1Inverted);
      map2.upload(_map2);
    }
  }

  if (Filter::GPU_ON) {
    Benchmark::open("OPENCL REMAP");
    cv::ocl::oclMat image_gpu;
    image_gpu.upload(input);
    cv::ocl::oclMat image_gpu_undist;

    cv::ocl::remap(image_gpu, image_gpu_undist, map1, map2, cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT);
    image_gpu_undist.download(img());
    // std::cerr<<"OPENCL"<<std::endl;

    Benchmark::close("OPENCL REMAP");
  } else {
    Benchmark::open("remap");
    cv::remap(input, img(), _map1Inverted, _map2, cv::INTER_LINEAR);
    Benchmark::close("remap");
  }
}
}
}
