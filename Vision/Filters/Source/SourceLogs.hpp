#pragma once

#include "Filters/Source/Source.hpp"
#include "FrameSource/ImageSequence.hpp"

// TODO: it seems this class should inherit of ImageSequence more than contain
// an ImageSequence

namespace Vision {
namespace Filters {

/**
 * SourceLogs
 *
 * No dependency filter providing frames coming from the specified folder
 */
class SourceLogs : public Source {
public:
  SourceLogs() : Source("SourceLogs") {}

  SourceLogs(const std::string &name, const std::string &logFile,
              int sIndex, Frequency::type frequency = Frequency::Auto);

  // JSON stuff
  virtual void fromJson(const Json::Value & v, const std::string & dir_name);
  virtual Json::Value toJson() const;
  virtual std::string getClassName() const override { return "SourceLogs"; }
  virtual int expectedDependencies() const override { return 0; }

  virtual Type getType() const override;

  std::string getImgName() const;
  int getIndex() const;

  bool isValid() const;

  void update() override;
  void previous() override;
  void updateImg();
  void setErrorImg(const std::string &errorMsg);

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  void openImageSequence();

private:
  int startIndex;
  std::string imagesFile;

  Utils::ImageSequence images;
};
}
}
