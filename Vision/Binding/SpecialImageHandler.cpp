#include "SpecialImageHandler.hpp"

namespace Vision
{
SpecialImageHandler::SpecialImageHandler()
  : name(""), baseWidth(0), baseHeight(0), scale(1.0), display(false), lastImg(), getter()
{
}
SpecialImageHandler::SpecialImageHandler(const std::string& name_, int baseWidth_, int baseHeight_, Getter getter_,
                                         double scale_, bool display_)
  : name(name_), baseWidth(baseWidth_), baseHeight(baseHeight_), scale(scale_), display(display_), getter(getter_)
{
}

int SpecialImageHandler::getSize() const
{
  return 3 * getWidth() * getHeight();
}

int SpecialImageHandler::getWidth() const
{
  return (int)(scale * baseWidth);
}

int SpecialImageHandler::getHeight() const
{
  return (int)(scale * baseHeight);
}
}  // namespace Vision
