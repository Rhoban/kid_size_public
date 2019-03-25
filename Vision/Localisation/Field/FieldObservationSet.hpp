#include "Localisation/Field/SerializableFieldObservation.hpp"

namespace Vision
{
namespace Localisation
{
class FieldObservationSet : public rhoban_utils::JsonSerializable
{
public:
  std::vector<SerializableFieldObservation*> observations;

  ~FieldObservationSet();

  void push(SerializableFieldObservation* obs);

  std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  void treatObservationNode(const Json::Value& v, const std::string& dir_name);
};
}  // namespace Localisation
}  // namespace Vision
