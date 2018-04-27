#pragma once

#include "Localisation/Field/FieldPosition.hpp"
#include "Localisation/Field/FieldPF.hpp"

#include "rhoban_unsorted/particle_filter/observation.h"
#include "Localisation/Field/ArenaCornerObservation.hpp"

#include <thread>
#include <vector>

class MoveScheduler;

// Forward declaration to reduce compile time
namespace Vision {
class Robocup;
namespace Localisation {
class CompassObservation;
class FieldPF;
class GoalObservation;
class TagsObservation;
}
namespace Utils {
class CameraState;
}


/// Charged of all the localisation services for the Robocup
///
/// It can be used in a separate thread
class LocalisationBinding {
private:
  typedef std::vector<rhoban_unsorted::Observation<Localisation::FieldPosition> * > ObservationVector;

public:
  LocalisationBinding(MoveScheduler * scheduler,
                      Robocup * vision_binding);

  ~LocalisationBinding();

  void run();
  void init();
  void step();

  void initRhIO();
  void importFromRhIO();
  void publishToRhIO();

  /// Return the timestamp associated to current time. If helpers is in fake
  /// mode, it uses the vision actual timeStamp
  ::rhoban_utils::TimeStamp getNowTS();

  /// Lock mutex and update information from the filter
  void importFiltersResults();

  /// Steal all the localisation informations provided by the vision
  void stealFromVision();

  std::vector<Localisation::GoalObservation *> extractGoalObservations();
  std::vector<Localisation::TagsObservation *> extractTagsObservations();
  std::vector<Localisation::CompassObservation *> extractCompassObservations();
  std::vector<Localisation::ArenaCornerObservation *> extractArenaCornerObservations();
  /// Use the provided informations in origin basis to create observations
  ObservationVector extractObservations();

  /// Tick the filter with the provided informations
  void updateFilter(const ObservationVector & obs);
  /// Tick the consistency watcher using provided informations
  void applyWatcher(const ObservationVector & obs);

  /// Ask for a reset of the particle filter
  void fieldReset(Localisation::FieldPF::ResetType type, float x = 0,
                  float y = 0, float noise = 0, float theta = 0,
                  float thetaNoise = 180);

  /// Publish information to the localisation service of the bounded scheduler
  void publishToLoc();

  cv::Mat getTopView(int width, int height);

  void setVisualCompassStatus(bool active);

  bool refereeAllowsToPlay();

  /// Link to the vision binding is required to retrieve information
  Robocup * vision_binding;

  /// Direct link to the scheduler
  MoveScheduler * scheduler;

  /// The particle filter containing the position of the robot
  Localisation::FieldPF * field_filter;

  /// Number of particles for the field
  int nb_particles_ff;

  /// Filters results
  rhoban_utils::Angle toGoal;
  Localisation::FieldPosition robot;
  double toGoalQ, robotQ;

  bool enableFieldFilter;
  bool isGoalKeeper;
  /// Handling timeStamps
  rhoban_utils::TimeStamp currTS, lastTS, lastFieldReset, lastUniformReset;


  /// Elapsed time since last reset [s]
  double elapsedSinceReset;

  /// Elapsed time since last uniform reset [s]
  double elapsedSinceUniformReset;

  /// Elapsed time since last convergence of the filter [s]
  double elapsedSinceConvergence;

  /// Is consistency enabled ?
  bool consistencyEnabled;
  /// Current consistence score (reset filters when it reaches zero)
  double consistencyScore;
  /// Amount removed from consistencyScore at each step
  double consistencyStepCost;
  /// Amount removed from consistencyScore for each bad observation
  double consistencyBadObsCost;
  /// Amount added to consistencyScore for each good observation
  double consistencyGoodObsGain;
  /// Amount reset
  double consistencyResetInterval;
  /// Noise at each step is increased up to consistencyMaxNoise
  double consistencyMaxNoise;

  /// Camera states
  Utils::CameraState * cs;

  /// Wished period between two ticks of the localisationBinding
  double period;

  // After a uniformReset, there is a temporary boost of exploration because
  // density of particles is lower near the position

  /// Maximal multiplier for exploration in boost mode
  double maxNoiseBoost;

  /// [s] After a full reset, duration of the noiseBoost
  double noiseBoostDuration;

  /// Status of visual compass
  bool isUsingVisualCompass;

  // VISION INFORMATION MEMORY
  // All informations are provided in world basis using [m]
  // Informations are cleared at every step of the LocalisationBinding

  /// Locations of the goals
  std::vector<cv::Point2f> goalsLocations;

  /// Indices of markers
  std::vector<int> markerIndices;

  /// Position of the center of the markers in the world
  std::vector<Eigen::Vector3d> markerPositions;

  /// Clipping infos
  std::vector<Vision::Filters::FieldBorderData> clipping_data;

  /// Orientations seen by the visual compass
  std::vector<double> compassOrientations;

  /// Quality of the compass observations
  std::vector<double> compassQuality;

  /// Number of compass observations acquired since start of VC phase
  int nbVCObs;

  /// After minVCObs, VC mode is disbled
  int minVCObs;

  /// Was last tick forbidden by the referee?
  ::rhoban_utils::TimeStamp lastForbidden;

  /// Is Localisation in a forbidden state
  bool isForbidden;

  /// Verbosity level
  int debugLevel;

private:
  /// The binding for RhIO variables
  RhIO::Bind* bind;

  /// Main thread running
  std::thread *_runThread;

  /// Locking access
  std::mutex filterMutex;

  /// Locking access to number of VC Observations
  std::mutex vcCounterMutex;
};

}
