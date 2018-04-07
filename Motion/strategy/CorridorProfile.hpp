#pragma once

#include <json/json.h>

#include <rhoban_utils/serialization/json_serializable.h>

/// This class is used to separate the field in different zones with different
/// weights based on the following scheme (example with 2 separations in x and y)
///
///                                x_limit[1]            x_limit[2]
///                                   \/                    \/
///  ---------------------------------------------------------------
/// |                                  |                     |      |
/// |     weights[2]                   |                     |  w   |
/// |                                  |                     |  e   |
/// |----------------------------------|y_limits[1]          |  i   |
/// |                                  |                     |  g   |
/// |     weights[1]                   |          weights[3] |  h   |
/// |                                  |                     |  t   |
/// |----------------------------------|y_limits[0]          |  s   |
/// |                                  |                     |  [   |
/// |     weights[0]                   |                     |  4   |
/// |                                  |                     |  ]   |
/// |                                  |                     |      |
///  ---------------------------------------------------------------
class CorridorProfile : public rhoban_utils::JsonSerializable {
public:
  CorridorProfile();

  double getWeight(double x, double y);

  virtual std::string getClassName() const override;
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value & json_value,
                        const std::string & dir_name) override;

private:
  std::vector<double> x_limits;
  std::vector<double> y_limits;
  std::vector<double> weights;
};
