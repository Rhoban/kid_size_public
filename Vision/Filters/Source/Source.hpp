#pragma once

#include "Filters/Filter.hpp"
namespace Vision {
namespace Filters {

/// Source abstract class: allow distinction between log and online sources
class Source : public Filter {
public:
  enum Type { Log, Online };

  Source(const std::string & name) : Filter(name) {}

  Source(const std::string &name, const Dependencies & dependencies,
         Frequency::type frequency) : Filter(name, dependencies, frequency) {}

  virtual Type getType() const = 0;
};
}
}
