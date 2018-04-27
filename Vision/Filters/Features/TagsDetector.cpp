#include "Filters/Features/TagsDetector.hpp"

#include "rhoban_utils/logging/logger.h"
#include "rhoban_utils/timing/benchmark.h"

#include <cvdrawingutils.h>
#include <vector>
#include <iostream>

using namespace aruco;
using namespace std;
using ::rhoban_utils::Benchmark;

static rhoban_utils::Logger out("TagsDetector");

namespace Vision {
namespace Filters {

TagsDetector::TagsDetector() : Filter("TagsDetector")
{
  //TODO: param in xml?
  CamParam.readFromXMLFile("camera_calib.yml");
  MDetector.setDictionary("ARUCO_MIP_36h12");
  // Getting default parameters
  detectionParam = MDetector.getParams();
}


void TagsDetector::setParameters() {
  /* Note : the default value are written below (to be binded to rhio if need be)
     _thresMethod = ADPT_THRES;
     _thresParam1 = _thresParam2 = 7;
     _cornerMethod = LINES;
     
     _thresParam1_range = 0;
     _markerWarpSize = 56;
     
     _minSize = 0.04;_maxSize = 0.95;_minSize_pix=25;
     _borderDistThres = 0.005; // corners at a distance from image boundary nearer than 2.5% of image are ignored
     _subpix_wsize=5;//window size employed for subpixel search (in vase you use _cornerMethod=SUBPIX
  */
  thresholdParam1 = ParamFloat(7, 0, 20);
  params()->define<ParamFloat>("thres1", &thresholdParam1);
  thresholdParam2 = ParamFloat(7, 0, 20);
  params()->define<ParamFloat>("thre2", &thresholdParam2);
  thresholdParamRange1 = ParamInt(0, 0, 20);
  params()->define<ParamInt>("thresRange", &thresholdParamRange1);

  minSize = ParamFloat(0.01, 0, 1);
  params()->define<ParamFloat>("minSize", &minSize);
  maxSize = ParamFloat(0.95, 0, 1);
  params()->define<ParamFloat>("maxSize", &maxSize);

  minSize_pix = ParamInt(7, 0, 1280);
  params()->define<ParamInt>("minSize_pix", &minSize_pix);
  
 //marker size in m
  markerSize = ParamFloat(0.09, 0, 1.0);
  params()->define<ParamFloat>("markerSize", &markerSize);
  
  subpix_wsize = ParamInt(1, 1, 25);
  params()->define<ParamInt>("subpix_wsize", &subpix_wsize);
  
  debugLevel = ParamInt(0, 0, 2);
  params()->define<ParamInt>("debugLevel", &debugLevel);
  period = ParamInt(1, 1, 100);
  params()->define<ParamInt>("period", &period);
}
  
void TagsDetector::process() {

  Markers.clear();
  const cv::Mat & srcImg = *(getDependency().getImg());

  // Importing parameters from rhio
  // Adaptative threshold is used since it is note modified
  detectionParam._thresParam1 = thresholdParam1;
  detectionParam._thresParam2 = thresholdParam2;
  detectionParam._thresParam1_range = thresholdParamRange1;
  detectionParam._minSize = minSize;
  detectionParam._maxSize = maxSize;
  detectionParam._minSize_pix = minSize_pix;
  detectionParam._subpix_wsize = subpix_wsize;
  // End function if we have not reached period
  periodCounter++;
  if (periodCounter < period) {
    return;
  }
  periodCounter = 0;

  // Copying image if necessary
  if (debugLevel > 0) {
    img() = srcImg.clone();
  } else {
    img() = srcImg;
  }

  // Setting parameters
  // TODO: parameters borderDistThres could be bused too
  //MDetector.setThresholdParams(thresholdParam1, thresholdParam2);
  // According to aruco documentation (and sourcecode), param2Range is not used yet
  //size_t param2Range = 0;
  //MDetector.setThresholdParamRange(thresholdParamRange1, param2Range);
  MDetector.setParams(detectionParam);
  Benchmark::open("Detecting Tags");
  Markers=MDetector.detect(img(), CamParam, markerSize);
  Benchmark::close("Detecting Tags");

  if (debugLevel >= 1) {
    Benchmark::open("Drawing Tags");
    // for each marker, draw info and its boundaries in the image
    for(unsigned int i = 0; i < Markers.size(); i++) {
      if (debugLevel >= 2)
        cout << Markers[i] << endl;
      Markers[i].draw(img(), cv::Scalar(0, 0, 255), 2);
    }
    // draw a 3d cube in each marker if there is 3d info
    if (CamParam.isValid() && markerSize != -1) {
      if (CamParam.CamSize != img().size()) {
        CamParam.resize(img().size());
        out.log("Resizing camera parameters");
      }
      for (unsigned int i = 0; i < Markers.size(); i++) {
        CvDrawingUtils::draw3dCube(img(), Markers[i], CamParam);
      }
    }

    if (debugLevel >= 2) {
      // draw a point where the middle of the tag is supposed to be. Using a barycenter for now, TODO better if need be.
      int avrgX = 0;
      int avrgY = 0;
      for (unsigned int markerId = 0; markerId < Markers.size(); markerId++) {
        for (unsigned int cornerId = 0; cornerId < Markers[markerId].size(); cornerId++) {
          std::cout << "Tag " << markerId << ", corner " << cornerId
                    << " : " << Markers[markerId][cornerId] << std::endl;
          avrgX = avrgX + Markers[markerId][cornerId].x;
          avrgY = avrgY + Markers[markerId][cornerId].y;
        }
      }
      avrgX = avrgX/4.0;
      avrgY = avrgY/4.0;
      cv::circle(img(), cv::Point2i(avrgX, avrgY), 3, cv::Scalar(0, 0, 200), -1);
    }
    
    Benchmark::close("Drawing Tags");
  }

}

vector< Marker > TagsDetector::getDetectedMarkers() const
{
  // returns a copy of the vector. From each Marker in the vector, we can use the members:
  // int id: unique id of the marker
  // float ssize: size of markers sides in m
  // cv::Mat Rvec and Tvec: matrices of rotation and translation respect to the camera
  return this->Markers;
}

int TagsDetector::expectedDependencies() const {
  return 1;
}

}
}
