#include "SearchController.h"
#include <ros/ros.h>
#include <string>

// ROS libraries
#include <angles/angles.h>

//define rover with int id
int id;

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  isInitiated=  false;
  currIndex=  0;
  deg=  0;
}


// Makes a path that spirals in a cubed fashion
void SearchController::cubedSpiralAlgorithm(float cx, float cy, float r, geometry_msgs::Pose2D currentLocation) 
{
    /*goals[0]= Pos2D();
    goals[0].x= cx;
    goals[0].y= cy;*/
    // Cubed Spiral Algorithm

    //Center robots to center of map
    cx -= getX(currentLocation.x);
    cy -= getY(currentLocation.y);

      for(int i= 0; i< GOAL_SIZE; i++)  {
        switch(i%4) {
          case 0: cx-=  (i+1)*r;  break;
          case 1: cy-=  (i+1)*r;  break;
          case 2: cx+=  (i+1)*r;  break;
          case 3: cy+=  (i+1)*r;  break;
        }

        goals[i]= Pos2D();
        goals[i].x= cx;
        goals[i].y= cy;
      }
}

//Initial Positions of Rovers
//{QPointF(0.0,1.0), QPointF(1.0,1.0), QPointF(1.0,0.0), QPointF(-1.0,0.0), QPointF(0.0,-1.0), QPointF(-1.0,-1.0)};

//Get Actual Center X
float SearchController::getX(float x)
{
  switch(id)
  {
    //Achiles
    case 0: return x;
    //Aeneas
    case 1: return x -1;
    //Ajax
    case 2: return x -1; 
  }  
}
//Get Actual Center Y
float SearchController::getY(float y)
{
  switch(id)
  {
    //Achiles
    case 0: return y - 1;
    //Aeneas
    case 1: return y - 1;
    //Ajax
    case 2: return y; 
  }  
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

 if(!isInitiated) {
   isInitiated= true;
 if(publishedName== "achilles"){id = 0;  cubedSpiralAlgorithm(0.0f, 0.0f, 1.0f, currentLocation);}
    else if(publishedName== "aeneas"){  id = 1; cubedSpiralAlgorithm(0.0f, 0.0f, 1.0f, currentLocation);}
    else if(publishedName== "ajax"){ id = 2; cubedSpiralAlgorithm(0.0f, 0.0f, 1.0f, currentLocation); }
 }
 //Test
 //ROS_INFO_STREAM("This is x: "<< getX(currentLocation.x)<< " Rover:  "<< publishedName);
 //ROS_INFO_STREAM("This is y: "<< getY(currentLocation.y)<< " Rover:  "<< publishedName);
    
  n=  goals[currIndex].x;
  m=  goals[currIndex].y;
  if(
    currentLocation.x>= n-EPSILON_SEARCH_RAD && currentLocation.x<= n+EPSILON_SEARCH_RAD &&
    currentLocation.y>= m-EPSILON_SEARCH_RAD && currentLocation.y<= m+EPSILON_SEARCH_RAD
  )  {
    currIndex++;
    currIndex%= GOAL_SIZE;
  }
  goalLocation.x = n;
  goalLocation.y = m;
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
