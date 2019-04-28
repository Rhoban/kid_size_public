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
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <utility>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <rhoban_utils/logging/logger.h>

static rhoban_utils::Logger logger("EverythingByDNN");

using namespace std;
using namespace rhoban_geometry;
using ::rhoban_utils::Benchmark;

namespace Vision
{
namespace Filters
{
// EverythingByDNN::EverythingByDNN() 
// {
// }

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
  // TODO
  // v["arch_path"] = arch_path;
  // v["weights_path"] = weights_path;
  return v;
}

void EverythingByDNN::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "model_path", &model_path);

  // updateNN();
}

int EverythingByDNN::expectedDependencies() const
{
  return 1;
}

void EverythingByDNN::updateNN()
{
  // TODO
  // // load the architecture of the model in json format
  // nn.load(arch_path, tiny_dnn::content_type::model, tiny_dnn::file_format::json);
  // // load the weights of the model in binary format
  // nn.load(weights_path, tiny_dnn::content_type::weights, tiny_dnn::file_format::binary);
}
  
std::pair<int, double> EverythingByDNN::getClass(cv::Mat patch)
{
  cv::Size patchSize = patch.size();

  if(patchSize.width != patchSize.height != 32) // TODO hardcoded sizes
    cv::resize(patch, patch, cv::Size(32, 32));
  cv::dnn::Blob in = cv::dnn::Blob::fromImages(patch);

  // --- TODO do that only once --- 
  auto importer = cv::dnn::createTensorflowImporter(model_path.c_str());
  cv::dnn::Net net;
  importer->populateNet(net);
  // ------------------------------

  
  net.setBlob(".img", in); 
  Benchmark::open("predict");
  net.forward();
  Benchmark::close("predict");

  cv::dnn::Blob prob = net.getBlob("tiny_model/fc2/fc2/Softmax");   //gather output of "prob" layer
  
  int classId;
  double classProb;
  
  cv::Mat probMat = prob.matRefConst().reshape(1, 1); //reshape the blob to 1x1000 matrix
  cv::Point classNumber;
  minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);
  classId = classNumber.x;
  logger.log("%d, %f", classId, classProb);
  
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
      throw std::runtime_error("BallByDNN:: number of rois does not match number of patches");

    for (size_t patch_id = 0; patch_id < rois.size(); patch_id++)
    {
      const cv::Mat& patch = patches[patch_id];
      const cv::RotatedRect& roi = rois[patch_id].second;

      std::pair<int, double> res = getClass(patch);
      

      bool isValid = res.second >= scoreThreshold;
      
      
      std::string s_class = std::to_string(res.first);
      std::vector<std::string> classNames;
      classNames.push_back("Empty");
      classNames.push_back("Ball");
      classNames.push_back("PostBase");

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
