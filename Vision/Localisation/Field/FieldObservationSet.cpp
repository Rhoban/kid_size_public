#include "Localisation/Field/FieldObservationSet.hpp"

#include "Localisation/Field/ArenaBorderObservation.hpp"
#include "Localisation/Field/ArenaCornerObservation.hpp"
#include "Localisation/Field/GoalObservation.hpp"

using namespace rhoban_unsorted;

namespace Vision {
namespace Localisation {

FieldObservationSet::~FieldObservationSet() {
  for (Observation<FieldPosition> *o : observations) {
    delete (o);
  }
}

void FieldObservationSet::push(SerializableFieldObservation *obs) {
  observations.push_back(obs);
}

std::string FieldObservationSet::getClassName() const {
  return "FieldObservationSet";
}

Json::Value FieldObservationSet::toJson() const {
  Json::Value v;
  for (SerializableFieldObservation *o : observations) {
    v.append(o->toFactoryJson());
  }
  return v;
}

void FieldObservationSet::fromJson(const Json::Value & v, const std::string & dir_name) {
  if (!v.isArray()) {
    throw rhoban_utils::JsonParsingError("FieldObservationSet::fromJson: expecting an array");
  }
  observations.clear();
  for (Json::ArrayIndex idx=0; idx < v.size(); idx++) {
    treatObservationNode(v[idx], dir_name);
  }
}

void FieldObservationSet::treatObservationNode(const Json::Value & v, const std::string & dir_name) {
  // TODO: replace by a factory
  std::string className = v["class name"].asString();
  SerializableFieldObservation *obs;
  if (className == "GoalObservation") {
    obs = new GoalObservation();
  } else if (className == "ArenaBorderObservation") {
    obs = new ArenaBorderObservation();
  } else if (className == "ArenaCornerObservation") {
    obs = new ArenaCornerObservation();
  } else {
    throw std::runtime_error("Unknown name for observation: '" + className +
                             "'");
  }
  obs->fromJson(v["content"], dir_name);
  observations.push_back(obs);
}
}
}
