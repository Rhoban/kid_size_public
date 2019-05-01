#include "Filters/Custom/EverythingByDNN.hpp"

#include "CameraState/CameraState.hpp"
#include "Filters/Patches/PatchProvider.hpp"
#include "Utils/RotatedRectUtils.hpp"
#include "Utils/Interface.h"
#include "Utils/OpencvUtils.h"
#include "Utils/ROITools.hpp"
#include "rhoban_utils/timing/benchmark.h"

#include "rhoban_geometry/circle.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <utility>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <rhoban_utils/logging/logger.h>

#include <hl_monitoring/field.h>

static rhoban_utils::Logger logger("EverythingByDNN");

using namespace std;
using namespace rhoban_geometry;
using ::rhoban_utils::Benchmark;

namespace Vision
{
namespace Filters
{
  
  std::map<std::string, hl_monitoring::Field::POIType> EverythingByDNN::stringToPOIEnum = {
    {"ArenaCorner", hl_monitoring::Field::POIType::ArenaCorner},
    {"LineCorner", hl_monitoring::Field::POIType::LineCorner},
    {"T", hl_monitoring::Field::POIType::T},
    {"X", hl_monitoring::Field::POIType::X},
    {"Center", hl_monitoring::Field::POIType::Center},
    {"PenaltyMark", hl_monitoring::Field::POIType::PenaltyMark},
    {"PostBase", hl_monitoring::Field::POIType::PostBase}
  };
  
EverythingByDNN::EverythingByDNN() : Filter("EverythingByDNN"), model_path("model.pb")
{
  // TODO load classes from json config file
  classNames.push_back("Empty");
  classNames.push_back("Ball");
  classNames.push_back("PostBase");
  classNames.push_back("LineCorner");
  classNames.push_back("PenaltyMark");
  classNames.push_back("Center");
  classNames.push_back("X");
  classNames.push_back("ArenaCorner");
  classNames.push_back("T");


  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("ArenaCorner", hl_monitoring::Field::POIType::ArenaCorner));
  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("LineCorner", hl_monitoring::Field::POIType::LineCorner));
  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("T", hl_monitoring::Field::POIType::T));
  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("X", hl_monitoring::Field::POIType::X));
  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("Center", hl_monitoring::Field::POIType::Center));
  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("PenaltyMark", hl_monitoring::Field::POIType::PenaltyMark));
  // stringToPOIEnum.insert(std::pair<std::string, hl_monitoring::Field::POIType>("PostBase", hl_monitoring::Field::POIType::PostBase));
}


void EverythingByDNN::setParameters()
{
  // debugLevel = ParamInt(0, 0, 1);
  scoreThreshold = ParamFloat(0.5, 0.0, 1.0);

  // params()->define<ParamInt>("debugLevel", &debugLevel);
  params()->define<ParamFloat>("scoreThreshold", &scoreThreshold);
}

std::string EverythingByDNN::getClassName() const
{
  return "EverythingByDNN";
}
  
Json::Value EverythingByDNN::toJson() const
{
  Json::Value v = Filter::toJson();
  v["model_path"] = model_path;
  return v;
}

void EverythingByDNN::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "model_path", &model_path);

  updateNN();
}

int EverythingByDNN::expectedDependencies() const
{
  return 1;
}

void EverythingByDNN::updateNN()
{
  importer = cv::dnn::createTensorflowImporter(model_path.c_str());
  importer->populateNet(net);
}
  
std::pair<int, double> EverythingByDNN::getClass(cv::Mat patch)
{
  cv::Size patchSize = patch.size();

  if(patchSize.width != patchSize.height != 32) // TODO hardcoded sizes
    cv::resize(patch, patch, cv::Size(32, 32));
  
  cv::dnn::Blob in = cv::dnn::Blob::fromImages(patch);
  
  net.setBlob(".img", in);
  
  Benchmark::open("predict");
  net.forward();
  Benchmark::close("predict");
  
  cv::dnn::Blob prob = net.getBlob("tiny_model/fc2/fc2/Softmax");   //gather output of "prob" layer
  // std::cout << prob << std::endl;
  int classId;
  double classProb;
  
  cv::Mat probMat = prob.matRefConst().reshape(1, 1); //reshape the blob to 1x1000 matrix
	
  std::cout << probMat << std::endl;
  cv::Point classNumber;
  minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);
  classId = classNumber.x;
  // logger.log("%d, %f", classId, classProb);

  return std::pair<int, double>(classId, classProb);
}
  
void EverythingByDNN::process()
{
  cv::Mat output;
  try
  {
    const PatchProvider& dep = dynamic_cast<const PatchProvider&>(getDependency());
    output = dep.getImg()->clone();
    const std::vector<cv::Mat>& patches = dep.getPatches();
    const std::vector<std::pair<float, cv::RotatedRect>> rois = dep.getRois();

    if (rois.size() != patches.size())
      throw std::runtime_error("EverythingByDNN:: number of rois does not match number of patches");

    for (size_t patch_id = 0; patch_id < rois.size(); patch_id++)
    {
      const cv::Mat& patch = patches[patch_id];
      const cv::RotatedRect& roi = rois[patch_id].second;

      std::pair<int, double> res = getClass(patch);

      bool isValid = res.second >= scoreThreshold;
      
      int int_class = res.first;
      std::string s_class = classNames.at(int_class);

      if(int_class != 0){// not Empty
	if(int_class == 1)// Ball
	  features_provider.pushBall(cv::Point2f(roi.center.x, roi.center.y));
	else
	  features_provider.pushPOI(stringToPOIEnum.at(s_class), cv::Point2f(roi.center.x, roi.center.y));
      }

      if(res.first == 0) // Empty
      {
	drawRotatedRectangle(output, roi, cv::Scalar(0, 0, 255), 2);
	// cv::putText(output, s_class, cv::Point(roi.center.x, roi.center.y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.8, cv::Scalar(0, 0, 255), 1.5, CV_AA);
      }
      else
      {      
	if (isValid)
	{
	  drawRotatedRectangle(output, roi, cv::Scalar(0, 255, 0), 2);
	  cv::putText(output, classNames.at(res.first), cv::Point(roi.center.x, roi.center.y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.8, cv::Scalar(0, 255, 0), 1.5, CV_AA);	
	}
	else
	{
	  drawRotatedRectangle(output, roi, cv::Scalar(0, 0, 255), 2);
	  cv::putText(output, classNames.at(res.first), cv::Point(roi.center.x, roi.center.y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.8, cv::Scalar(0, 0, 255), 1.5, CV_AA);
	}
      }

    }
  }
  catch (const std::bad_alloc& exc)
  {
    std::ostringstream oss;
    oss << "Pipeline structure is invalid: "
        << "filter " << name << " dependency is not a PatchProvider" << std::endl;
    throw std::runtime_error(oss.str());
  }
  img() = output;
  
}
}  // namespace Filters
}  // namespace Vision
