#include "Recorder.hpp"

#include "RhIO.hpp"

#include <opencv2/opencv.hpp>

namespace Vision
{
namespace Filters
{
Recorder::Recorder() : Filter("Recorder"), activated(false), image_no(0)
{
}

Recorder::~Recorder()
{
}

std::string Recorder::getClassName() const
{
  return "Recorder";
}

void Recorder::process()
{
  updateControl();
  img() = *(getDependency().getImg());
  if (!activated)
  {
    if (memory.size() != 0)
    {
      for (size_t i = 0; i < memory.size(); i++)
      {
        // TODO format properly with setwidth
        std::ostringstream oss;
        oss << "Image_" << i << ".png";
        cv::imwrite(oss.str(), memory[i]);
      }
      memory.clear();
    }
    image_no = 0;
    return;
  }
  memory.push_back(img().clone());
  // Save next image
  image_no++;
}

void Recorder::initControl()
{
  std::string activated_path = rhio_path + getName() + "/activated";
  if (RhIO::Root.getValueType(activated_path) != RhIO::ValueType::NoValue)
    return;
  RhIO::Root.newBool(activated_path)->defaultValue(activated);
}

void Recorder::updateControl()
{
  initControl();
  std::string activated_path = rhio_path + getName() + "/activated";
  activated = RhIO::Root.getValueBool(activated_path).value;
}
}  // namespace Filters
}  // namespace Vision
