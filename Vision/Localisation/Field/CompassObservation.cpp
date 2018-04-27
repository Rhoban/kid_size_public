#include "CompassObservation.hpp"

#include "Field/Field.hpp"

#include "RhIO.hpp"

#include <rhoban_utils/logging/logger.h>

#include <string>
#include <sstream>
#include <cstdlib>

static rhoban_utils::Logger logger("CompassObservation");

using namespace rhoban_utils;

namespace Vision {
namespace Localisation {

Angle CompassObservation::offset(0.0);

double CompassObservation::pError = 0.01;
double CompassObservation::maxError = 45;
double CompassObservation::sigmoidOffset = 0.01;
double CompassObservation::sigmoidLambda = 20;

CompassObservation::CompassObservation()
  : obs(0.0)
{
}


CompassObservation::CompassObservation(Angle compassValue)
{
  obs = compassToField(compassValue);
  logger.log("new observation: %f (raw value: %f, offset: %f)",
             obs.getSignedValue(), compassValue.getSignedValue(), offset.getSignedValue());
}

Angle CompassObservation::compassToField(Angle compassAngle)
{
  return offset - compassAngle;
}

void CompassObservation::setReference(Angle dirToOppGoal)
{
  offset = dirToOppGoal;
}

double CompassObservation::potential(const FieldPosition & p) const {
  // Computing the absolute difference of angle between what was expected and
  // what was received;
  double absDiffAngle = abs((p.getOrientation() - obs).getSignedValue());
  return sigmoidScore(absDiffAngle, maxError, pError, sigmoidOffset, sigmoidLambda);
}

void CompassObservation::bindWithRhIO()
{
  RhIO::Root.newFloat("/localisation/field/CompassObservation/offset")
    ->defaultValue(offset.getSignedValue())
    ->minimum(-180.0)->maximum(180.0)
    ->comment("The magneto azimuth value when facing the adversary goal");
  RhIO::Root.newFloat("/localisation/field/CompassObservation/pError")
    ->defaultValue(pError)
    ->minimum(0.0)->maximum(1.0)
    ->comment("The false positive probability");
  RhIO::Root.newFloat("/localisation/field/CompassObservation/maxError")
    ->defaultValue(maxError)
    ->minimum(0.0)->maximum(180)
    ->comment("The maximum angle difference between expectation and observation");
  RhIO::Root.newFloat("/localisation/field/CompassObservation/sigmoidOffset")
    ->defaultValue(sigmoidOffset)
    ->minimum(0.0)->maximum(1.0)
    ->comment("The value at which dScore/dx is lambda, with dx = dAngle/maxAngle");
  RhIO::Root.newFloat("/localisation/field/CompassObservation/sigmoidLambda")
    ->defaultValue(sigmoidLambda)
    ->minimum(0.0)->maximum(1000.0)
    ->comment("Cf. sigmoidOffset");
}

void CompassObservation::importFromRhIO()
{
  RhIO::IONode & node = RhIO::Root.child("localisation/field/CompassObservation");
  offset = node.getValueFloat("offset").value;
  pError  = node.getValueFloat("pError").value;
  maxError  = node.getValueFloat("maxError").value;
  sigmoidOffset  = node.getValueFloat("sigmoidOffset").value;
  sigmoidLambda  = node.getValueFloat("sigmoidLambda").value;
}

void CompassObservation::exportToRhIO()
{
  RhIO::IONode & node = RhIO::Root.child("localisation/field/CompassObservation");
  node.setFloat("offset", offset.getSignedValue());
  node.save("rhio/localisation/field/CompassObservation/");
}

Angle CompassObservation::getOffset()
{
  return offset;
}

void CompassObservation::setOffset(Angle offset_)
{
  offset = offset_;
  exportToRhIO();
}


std::string CompassObservation::getClassName() const
{
  return "CompassObservation";
}

Json::Value CompassObservation::toJson() const
{
  Json::Value v;
  v["obs"] = obs.getSignedValue();
  return v;
}

void CompassObservation::fromJson(const Json::Value & v, const std::string & dir_name)
{
  rhoban_utils::tryRead(v,"obs",&obs);
}

double CompassObservation::getMinScore() const
{
  return pError;
}

std::string CompassObservation::toStr() const
{
  std::ostringstream oss;
  oss << "[CompassObservation: " << obs.getSignedValue() << "°]";
  return oss.str();
}


}
}
