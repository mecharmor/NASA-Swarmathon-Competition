#include "SearchController.h"
#include <ros/ros.h>
#include <string>

// ROS libraries
#include <angles/angles.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  isInitiated=  false;
  currIndex=  0;
  deg=  0;
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation, std::string publishedName) {
  double n=0.0;
  double m=0.0;

  //ROS_INFO_STREAM("COS: published name: "<<publishedName);

  geometry_msgs::Pose2D goalLocation;

  //select new position 50 cm from current location
  //goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
  //goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
  if(publishedName=="achilles"){ //black rover
    if(!isInitiated)  {
      for(int i= 0; i< GOAL_SIZE; i++)  {
        goals[i]= Pos2D();
        goals[i].x= ((i<= 12) ? -6 : i-20);
        goals[i].y= ((i<= 12) ? i-6 : 5);
      }
      
      // Spiral algorithm
      //deg=  135;

      isInitiated=  true;
    }
    // Spiral algorithm
    /*
    n=  cos(deg*CONVERT_TO_RADS);
    m=  sin(deg*CONVERT_TO_RADS);
    ROS_INFO_STREAM("DERPALERP "<< deg);
    deg=  (deg+15)%360;
    */
    
    n=  goals[currIndex].x;
    m=  goals[currIndex].y;
    if(
      currentLocation.x>= n-EPSILON_SEARCH_RAD && currentLocation.x<= n+EPSILON_SEARCH_RAD &&
      currentLocation.y>= m-EPSILON_SEARCH_RAD && currentLocation.y<= m+EPSILON_SEARCH_RAD
    )  {
      currIndex++;
      currIndex%= GOAL_SIZE;
    }
    ROS_INFO_STREAM("Zachattack "<< currIndex);
    
    /*
    if(currentLocation.x > -6.0 && currentLocation.x < -4.0 && currentLocation.y < -1.0 && currentLocation.y > -2.0){
      n = 0.0;
      m = 0.0;
    }else{
      n=-5.0;
      m=-1.0;
    }*/
  }
  if(publishedName=="aeneas"){ //yellow rover
    n=5.0;
    m=-1.0;
  }
  if(publishedName=="ajax"){ //white rover
    n=-1.0;
    m=-5.0;
  }
  goalLocation.x = n;
  goalLocation.y = m;

  //ROS_INFO_STREAM("COS: Cur Loc x: "<< currentLocation.x);
  //ROS_INFO_STREAM("COS: Cur Loc y: "<< currentLocation.y);
  //ROS_INFO_STREAM("COS: Cur Loc theta : "<< currentLocation.theta);


  //select new heading from Gaussian distribution around current heading
  //goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

  double myHeading = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
  goalLocation.theta = myHeading;

 // ROS_INFO_STREAM("COS: aTan2 : "<< myHeading);

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
