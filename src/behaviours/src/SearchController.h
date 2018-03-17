#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include <cmath>

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetRoverName(string name);
  void SetSuccesfullPickup();

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;
  string roverName;

  // Search mode will determine what algo to run
  // 0 == random walk
  // 1 == COS waypoint
  int searchMode = 2;
  

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
};

#endif /* SEARCH_CONTROLLER */
