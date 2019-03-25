#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * ColorConverter
 *
 * Convert a frame from a color space to another
 */
class ColorConverter : public Filter
{
public:
  ColorConverter() : Filter("ColorConverter")
  {
  }

  // Json stuff
  virtual void fromJson(const Json::Value& v, const std::string& dir_name);
  virtual Json::Value toJson() const;
  virtual std::string getClassName() const override
  {
    return "ColorConverter";
  }

protected:
  std::string conversion;

  /**
   * @Inherit
   */
  virtual void process() override;

  int getConvCode();

private:
  static std::map<std::string, int> strToCVCode;

  static void buildMap();
};
}  // namespace Filters
}  // namespace Vision
