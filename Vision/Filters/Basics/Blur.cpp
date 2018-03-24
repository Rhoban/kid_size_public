#include "Filters/Basics/Blur.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include "rhoban_utils/timing/benchmark.h"
#include "opencv2/ocl/ocl.hpp"

using rhoban_utils::Benchmark;

namespace Vision {
namespace Filters {

void Blur::setParameters() {
  kWidth = ParamInt(3, 1, 25);
  kHeight = ParamInt(3, 1, 25);
  params()->define<ParamInt>("kernel width", &kWidth);
  params()->define<ParamInt>("kernel height", &kHeight);
}

void Blur::process() {
  cv::Mat src = *(getDependency().getImg());

  if (Filter::GPU_ON) {
    Benchmark::open("OPENCL blur");
    cv::ocl::oclMat image_gpu;
    image_gpu.upload(src);
    cv::ocl::oclMat image_gpu_blur;
    cv::ocl::blur(image_gpu, image_gpu_blur, cv::Size(kWidth, kHeight));
    image_gpu_blur.download(img());
    Benchmark::close("OPENCL blur");
  } else {
    blur(src, img(), cv::Size(kWidth, kHeight));
  }
}
}
}
