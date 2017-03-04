#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <string>

#define GOAL_SIZE 20
#define EPSILON_SEARCH_RAD 1
#define CONVERT_TO_RADS (3.14159/180)

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
struct Pos2D  {
  float x;
  float y;
};

class SearchController {

  public:

    SearchController();

    // Makes a path that spirals in a cubed fashion
    void cubedSpiralAlgorithm(float cx, float cy, float r, geometry_msgs::Pose2D currentLocation);

    //return an x that matches the grid for all rovers
    float getX(float x);
    float getY(float y);

    // performs search pattern
    //geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation);
    geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation, std::string publishedName);


    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

  private:

    random_numbers::RandomNumberGenerator* rng;
    //The Cornerstone Algorithm (Array goal Corner sweep search algorithm)
    Pos2D goals[GOAL_SIZE];
    bool  isInitiated;
    int currIndex;
    // Sprial search algorithm
    int deg; // Degrees
};

#endif /* SEARCH_CONTROLLER */
