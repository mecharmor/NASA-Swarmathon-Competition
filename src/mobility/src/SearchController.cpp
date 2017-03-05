#include "SearchController.h"
#include <ros/ros.h>
#include <string>
#include <math.h> /*sqrt*/

// ROS libraries
#include <angles/angles.h>

SearchController::SearchController() {
  //default constructor
  rng = new random_numbers::RandomNumberGenerator();

  isInitiated=  false;
  currIndex = 0;
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation, std::string publishedName) {
  //Pos2D is a simple (x,y) stuct found in the header file
  Pos2D resultPos;
  name = publishedName;
  // Pose2D is a ROS struct (x,y,heading)
  geometry_msgs::Pose2D goalLocation;


  if(publishedName=="achilles"){ //black rover

    // set waypoints with respect to the board origin
    // hopefully programmitically and in the constructor!
    goals[0].x = -3.0f;
    goals[0].y = -1.0f;
    goals[1].x = -3.0f;
    goals[1].y = 1.0f;
    goals[2].x = -1.0f;
    goals[2].y = 1.0f;


  }
  if(publishedName=="aeneas"){ //yellow rover
    // goal locations using board coordinates
    goals[0].x = 3.0f;
    goals[0].y = 1.0f;
    goals[1].x = 3.0f;
    goals[1].y = 3.0f;
    goals[2].x = 1.0f;
    goals[2].y = 1.0f;

  }
  if(publishedName=="ajax"){ //white rover
    // waypoints for ajax
    goals[0].x = 1.0f;
    goals[0].y = -3.0f;
    goals[1].x = -3.0f;
    goals[1].y = -3.0f;
    goals[2].x = 0.0f;
    goals[2].y = -1.0f;

  }

  // goal location should be relative to board coordinates
  resultPos = nextGoal(publishedName, currentLocation);

  goalLocation.x = resultPos.x;
  goalLocation.y = resultPos.y;

  double myHeading = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
  goalLocation.theta = myHeading;

  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;
  Pos2D resultPos;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
  

  return newGoalLocation;
}

Pos2D SearchController::nextGoal(std::string publishedName,geometry_msgs::Pose2D currentLocation){
  Pos2D goal;
  double _distance;
  if(!isInitiated){
    //The first point on the board to go to here:
    goal.x=goals[0].x;
    goal.y=goals[0].y;
    isInitiated=  true;
  }else{
    _distance = sqrt(pow(currentLocation.x-goals[currIndex].x,2.0)+pow(currentLocation.y-goals[currIndex].y,2.0));
    ROS_INFO_STREAM("COS: published name: "<<publishedName<<" distance "<<_distance << " to "<< currIndex);

    // go to next waypoint when goal is reached
    if(_distance<1.5){
      currIndex++;
      currIndex = currIndex%3;
      ROS_INFO_STREAM("NEW WAYPOINT "<<currIndex << " for "<< publishedName);
      goal.x=goals[currIndex].x;
      goal.y=goals[currIndex].y;
    }
  }

  if(publishedName=="achilles"){ //black rover
      // This rover starts at (0,1) so subtract 1 from goalLocation.y 
    goal.y-=1.0;
  }
  if(publishedName=="aeneas"){ //yellow rover
    // This rover starts at (1,1) so subtract 1 from goalLocation.y and x
    goal.x-=1.0;
    goal.y-=1.0;
  }
  if(publishedName=="ajax"){ //white rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    goal.x-=1.0;
  }

  return goal;
}


