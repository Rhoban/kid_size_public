#pragma once

#include "FieldPosition.hpp"
#include "rhoban_utils/serialization/json_serializable.h"

#include "rhoban_unsorted/particle_filter/bounded_score_observation.h"

// Ensure that the particle is inside of the field

namespace Vision
{
namespace Localisation
{
class SerializableFieldObservation : public rhoban_unsorted::BoundedScoreObservation<FieldPosition>,
                                     public rhoban_utils::JsonSerializable
{
};
}  // namespace Localisation
}  // namespace Vision
