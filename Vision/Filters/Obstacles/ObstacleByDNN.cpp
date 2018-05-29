#include "Filters/Obstacles/ObstacleByDNN.hpp"

#include "Filters/Patches/PatchProvider.hpp"
#include "Utils/RotatedRectUtils.hpp"
#include "Utils/OpencvUtils.h"
#include "Utils/ROITools.hpp"
#include "rhoban_utils/timing/benchmark.h"
#include "rhoban_utils/util.h"

#include "rhoban_geometry/circle.h"

#include <opencv2/imgproc/imgproc.hpp>

using ::rhoban_utils::Benchmark;
using namespace std;

static void patchToNN(const cv::Mat & patch,
                      double minv,
                      double maxv,
                      int w,
                      int h,
                      tiny_dnn::vec_t & data)
{
    if (patch.data == nullptr) return; // cannot open, or it's not an image
    cv::Mat resized,res;
    cv::resize(patch, resized, cv::Size(w, h), .0, .0,cv::INTER_AREA);
    data.resize(w*h*resized.channels(), minv);
    resized.copyTo(res);
    if (res.channels() == 3) {
      // Splitting channels
      cv::Mat ch1, ch2, ch3;
      vector<cv::Mat> channels(3);
      cv::split(res, channels);
      ch1 = channels[0];
      ch2 = channels[1];
      ch3 = channels[2];
      // Reformat data
      int j=0;
      
      cv::Size sz = ch1.size();
      int size=sz.height*sz.width;
    
      for(int i=0; i<size;i++)
        data[j*size+i]=minv+(maxv-minv)*ch1.data[i]/255.0;
      j++;
      for(int i=0; i<size;i++)
        data[j*size+i]=minv+(maxv-minv)*ch2.data[i]/255.0;
      j++;
      for(int i=0; i<size;i++)
        data[j*size+i]=minv+(maxv-minv)*ch3.data[i]/255.0;
    } else if (res.channels() == 1) {
      cv::Size sz = res.size();
      int size=sz.height*sz.width;
      for(int i=0; i<size;i++)
        data[i]=minv+(maxv-minv)*res.data[i]/255.0;
    }
}

namespace Vision {
namespace Filters {

ObstacleByDNN::ObstacleByDNN() : ObstacleProvider("ObstacleByDNN"),
                                 arch_path("arch.json"),
                                 weights_path("weights.data")
{
}


void ObstacleByDNN::setParameters()
{
  debugLevel = ParamInt(0,0,1);
  scoreThreshold = ParamFloat(0.5,0.01,1.0);

  params()->define<ParamInt>("debugLevel", &debugLevel);
  params()->define<ParamFloat>("scoreThreshold", &scoreThreshold);
}

std::string ObstacleByDNN::getClassName() const {
  return "ObstacleByDNN";
}
Json::Value ObstacleByDNN::toJson() const
{
  Json::Value v = Filter::toJson();
  v["arch_path"] = arch_path;
  v["weights_path"] = weights_path;
  return v;
}

void ObstacleByDNN::fromJson(const Json::Value & v, const std::string & dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"arch_path",&arch_path);
  rhoban_utils::tryRead(v,"weights_path",&weights_path);

  updateNN();
}

int ObstacleByDNN::expectedDependencies() const {
  return 1;
}

void ObstacleByDNN::updateNN()
{
  // load the architecture of the model in json format
  nn.load(arch_path, tiny_dnn::content_type::model, tiny_dnn::file_format::json);
  // load the weights of the model in binary format
  nn.load(weights_path, tiny_dnn::content_type::weights, tiny_dnn::file_format::binary);
}

double ObstacleByDNN::getScore(const cv::Mat & patch)
{
  tiny_dnn::vec_t data;

  Benchmark::open("resize");
  int width = nn[0]->in_data_shape()[0].width_;
  int height = nn[0]->in_data_shape()[0].height_;
  patchToNN(patch, -1.0, 1.0, width, height, data);
  Benchmark::close("resize");

  Benchmark::open("predict");
  auto res = nn.predict(data);
  Benchmark::close("predict");

  return res[1];
}

void ObstacleByDNN::process() {

  clearObstaclesData();

  cv::Mat output;

  try {
    const PatchProvider & dep = 
      dynamic_cast<const PatchProvider &>(getDependency());

    Benchmark::open("Cloning src");
    const cv::Mat & src = *(dep.getImg());
    if (src.channels() == 1) {
      output = cv::Mat(src.size(), CV_8UC3);
      cv::cvtColor(src, output, CV_GRAY2BGR);
    } else if (src.channels() == 3) {
      output = dep.getImg()->clone();
    } else {
      throw std::logic_error(DEBUG_INFO + " invalid number of channels in image");
    }
    Benchmark::close("Cloning src");

    const std::vector<cv::Mat> & patches = dep.getPatches();
    const std::vector<std::pair<float, cv::RotatedRect>> rois = dep.getRois();

    if (rois.size() != patches.size()) {
      throw std::runtime_error("ObstacleByDNN:: number of rois does not match number of patches");
    }

    if (debugLevel > 0) {
      std::cout << "Filter: " << name << std::endl;
    }

    Benchmark::open("Analyzing patches");
    for (size_t patch_id = 0; patch_id < rois.size(); patch_id++) {
      const cv::Mat & patch = patches[patch_id];
      const cv::RotatedRect & roi = rois[patch_id].second;

      double score = getScore(patch);

      if (debugLevel > 0) {
        std::cout << "\t" << Utils::toRect(roi) << " score: " << score << std::endl;
      }

      bool isValid = score > scoreThreshold;

      if (isValid) {
        pushObstacle(roi.center.x, roi.center.y, output);
        drawRotatedRectangle(output, roi, cv::Scalar(0,255,0), 2);
      }
      else {
        drawRotatedRectangle(output, roi, cv::Scalar(0,0,255), 2);
      }
    }
    Benchmark::close("Analyzing patches");
  }
  catch (const std::bad_alloc & exc) {
    std::ostringstream oss;
    oss << "Pipeline structure is invalid: "
        << "filter " << name << " dependency is not a PatchProvider" << std::endl;
    throw std::runtime_error(oss.str());
  }
  img() = output;
}
}
}
