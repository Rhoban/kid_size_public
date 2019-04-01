#pragma once

#include <opencv2/core/core.hpp>

namespace Vision
{
namespace Utils
{
class ImageSequence
{
private:
  int frameNo, nextFrameNo;
  int64_t clock_offset;
  cv::Mat img;
  std::string prefix;
  std::string suffix;
  std::vector<std::string> images;
  std::vector<unsigned long> timestamps;

  void update();

public:
  ImageSequence();
  /**
   * @see loadImages
   */
  ImageSequence(const std::string& fileName);

  std::string imgFileName() const;
  std::string imgOriginalName() const;
  const cv::Mat& getImg() const;
  unsigned long getTimestamp() const;

  /// Load a set of images from a csv file containing the list of image
  /// Images are not stored in memory, but path to images is loaded.
  /// File can easily be created by using:
  /// ls <folder>/*.jpg > filename
  /// or something similar
  void loadImages(const std::string& fileName);

  /* Load previous or next image, throw exception if operation cannot be
   * done (beginning/end of imageSequence)
   */
  void previousImg();
  void nextImg();

  /**
   * Move to given index and load image
   */
  void setIndex(int index);
  int getIndex() const
  {
    return frameNo;
  }

  /**
   * Return the total number of images available in the sequence
   */
  int nbImages() const;

  // Return true if current Image is the first/last
  bool isFirst() const;
  bool isLast() const;

  bool isValid() const;

  int64_t getClockOffset() const;
};
}  // namespace Utils
}  // namespace Vision
