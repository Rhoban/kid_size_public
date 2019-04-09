#pragma once

#include "Filters/Source/Source.hpp"
#include "FrameSource/ImageSequence.hpp"

// TODO: it seems this class should inherit of ImageSequence more than contain
// an ImageSequence

namespace Vision
{
namespace Filters
{
/**
 * SourceLogs
 *
 * No dependency filter providing frames coming from the specified folder
 */
class SourceLogs : public Source
{
public:
  SourceLogs() : Source("SourceLogs")
  {
  }

  // JSON stuff
  virtual void fromJson(const Json::Value& v, const std::string& dir_name);
  virtual Json::Value toJson() const;
  virtual std::string getClassName() const override
  {
    return "SourceLogs";
  }
  virtual int expectedDependencies() const override
  {
    return 0;
  }

  virtual Type getType() const override;

  std::string getImgName() const;
  int getIndex() const;

  bool isValid() const;

  void update() override;
  void previous() override;
  void updateImg();

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  void openImageSequence();

  int64_t getClockOffset() const override;

private:
  int startIndex;
  std::string imagesFile;

  Utils::ImageSequence images;
};
}  // namespace Filters
}  // namespace Vision
