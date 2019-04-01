#include <stdexcept>
#include <fstream>
#include <iostream>

#include "ImageSequence.hpp"
#include "Exceptions.hpp"

#include <opencv2/highgui/highgui.hpp>

#include "rhoban_utils/util.h"

using namespace cv;
using namespace std;

namespace Vision
{
namespace Utils
{
ImageSequence::ImageSequence() : frameNo(-1), nextFrameNo(0), suffix()
{
}

std::string ImageSequence::imgFileName() const
{
  ostringstream oss;
  oss << prefix << imgOriginalName() << suffix;
  return oss.str();
}

std::string ImageSequence::imgOriginalName() const
{
  if (frameNo < 0)
  {
    throw StreamEndException(DEBUG_INFO + "Index before start of image");
  }
  if (frameNo >= (int)images.size())
  {
    throw StreamEndException(DEBUG_INFO + "Index after last image");
  }
  return images[frameNo];
}

void ImageSequence::setIndex(int index)
{
  if (index >= (int)images.size())
    throw StreamEndException(DEBUG_INFO + "End of stream has been reached");
  if (index < 0)
    throw StreamEndException(DEBUG_INFO + "Negative index in imageSequence");
  nextFrameNo = index;
}

void ImageSequence::update()
{
  frameNo = nextFrameNo;
  img = imread(imgFileName());
  if (img.data == NULL)
  {
    throw std::runtime_error("Failed to read: '" + imgFileName() + "'");
  }
  nextFrameNo = frameNo + 1;
}

const Mat& ImageSequence::getImg() const
{
  return img;
}

void ImageSequence::loadImages(const std::string& fileName)
{
  images.clear();
  ifstream in;
  in.open(fileName);
  // Open file
  if (!in.is_open())
  {
    throw std::runtime_error("Invalid config file for ImageSequence: '" + fileName + "'");
  }
  // Read file
  std::string line;
  bool first_line = true;
  while (getline(in, line))
  {
    std::vector<std::string> values;
    // First line might contain clock_offset information
    if (first_line)
    {
      if (line.find("clock_offset:") != std::string::npos)
      {
        rhoban_utils::split(line, ':', values);
        clock_offset = std::stol(values[1]);
        continue;
      }
      first_line = false;
    }
    rhoban_utils::split(line, ',', values);
    timestamps.push_back(std::stoul(values[0]));
    images.push_back(values[1]);
  }
  in.close();
  // Update prefix
  int lastSeparator = -1;
  for (size_t i = 0; i < fileName.size(); i++)
  {
    if (fileName[i] == '/')
      lastSeparator = i;
  }
  if (lastSeparator < 0)

    prefix = "";
  else
    prefix = fileName.substr(0, lastSeparator + 1);
}

void ImageSequence::previousImg()
{
  setIndex(frameNo - 1);
  update();
}

void ImageSequence::nextImg()
{
  update();
}

bool ImageSequence::isFirst() const
{
  return (frameNo == 0);
}

bool ImageSequence::isLast() const
{
  return (frameNo + 1 == (int)images.size());
}

bool ImageSequence::isValid() const
{
  return (frameNo >= 0 && frameNo < (int)images.size());
}

unsigned long ImageSequence::getTimestamp() const
{
  return timestamps[frameNo];
}

int64_t ImageSequence::getClockOffset() const
{
  return clock_offset;
}

}  // namespace Utils
}  // namespace Vision
