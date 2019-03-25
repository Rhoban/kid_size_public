#include "Filters/Basics/Histogram.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision
{
namespace Filters
{
Histogram::Histogram() : Filter("Histogram")
{
}

void Histogram::setParameters()
{
  bins = ParamInt(256, 0, 256);
  params()->define<ParamInt>("bins", &bins);
}

void Histogram::process()
{
  cv::Mat src = *(getDependency().getImg());

  std::vector<cv::Mat> yuv;
  split(src, yuv);

  int histSize = bins;

  // Set the ranges
  float range[] = { 1, 256 };
  const float* histRange = { range };

  bool uniform = true;
  bool accumulate = false;

  cv::Mat yHist, uHist, vHist;

  /// Compute the histograms:
  cv::calcHist(&yuv[0], 1, 0, cv::Mat(), yHist, 1, &histSize, &histRange, uniform, accumulate);
  cv::calcHist(&yuv[1], 1, 0, cv::Mat(), vHist, 1, &histSize, &histRange, uniform, accumulate);
  cv::calcHist(&yuv[2], 1, 0, cv::Mat(), uHist, 1, &histSize, &histRange, uniform, accumulate);

  // Draw the histograms
  int hist_w = src.cols;
  int hist_h = src.rows;
  int bin_w = cvRound((double)hist_w / histSize);

  img() = cv::Mat(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

  // Normalize the result to [ 0, img().rows ]
  cv::normalize(yHist, yHist, 0, img().rows, cv::NORM_MINMAX, -1, cv::Mat());
  cv::normalize(uHist, uHist, 0, img().rows, cv::NORM_MINMAX, -1, cv::Mat());
  cv::normalize(vHist, vHist, 0, img().rows, cv::NORM_MINMAX, -1, cv::Mat());

  // Draw for each channel
  for (int i = 1; i < histSize; i++)
  {
    line(img(), cv::Point(bin_w * (i - 1), hist_h - cvRound(yHist.at<float>(i - 1))),
         cv::Point(bin_w * (i), hist_h - cvRound(yHist.at<float>(i))), cv::Scalar(255, 255, 255), 2, 8, 0);
    line(img(), cv::Point(bin_w * (i - 1), hist_h - cvRound(uHist.at<float>(i - 1))),
         cv::Point(bin_w * (i), hist_h - cvRound(uHist.at<float>(i))), cv::Scalar(0, 255, 0), 2, 8, 0);
    line(img(), cv::Point(bin_w * (i - 1), hist_h - cvRound(vHist.at<float>(i - 1))),
         cv::Point(bin_w * (i), hist_h - cvRound(vHist.at<float>(i))), cv::Scalar(0, 0, 255), 2, 8, 0);
  }
}
}  // namespace Filters
}  // namespace Vision
