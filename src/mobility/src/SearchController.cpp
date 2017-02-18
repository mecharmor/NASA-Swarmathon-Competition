#include "SearchController.h"
#include <ros/ros.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;

  //select new heading from Gaussian distribution around current heading
  //goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
  goalLocation.theta = currentLocation.theta+1;

  ROS_INFO_STREAM("COS: Cur Loc x: "<< currentLocation.x);
  ROS_INFO_STREAM("COS: Cur Loc y: "<< currentLocation.y);

  //select new position 50 cm from current location
  //goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
//goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

  goalLocation.x = 3;
  goalLocation.y = 3;

  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}
