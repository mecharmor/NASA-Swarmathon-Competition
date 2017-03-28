#include "SearchController.h"
#include <ros/ros.h>
#include <string>
#include <math.h> /*sqrt*/

// ROS libraries
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>


float clamp(float val, float min, float max);
bool isOutOfBounds(float x, float y);

float clamp(float val, float min, float max)  {
  if(val< min)
    return min;
  if(val> max)
    return max;
  return val;
}
bool isOutOfBounds(float x, float y)  {
  if(x< -6.0f || x> 6.0f)
    return true;
  if(y< -6.0f || y> 6.0f)
    return true;
  
  return false;
}


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
  // Pose2D is a ROS struct (x,y,heading)
  geometry_msgs::Pose2D goalLocation;

  if(!isInitiated)  {
    isInitiated=  true;
    name = publishedName;

    if(publishedName=="achilles"){ //black rover
      
      // set waypoints with respect to the board origin
      // rough start to try to avoid other rovers at start
      goals[0].x= 0.0f;
      goals[0].y= 0.0f;
      goals[1].x= -2.0f;
      goals[1].y= 1.0f;

      float r= 1.0f;
      bool clockwise=true;
      goals[2].x = r*cos(210*PI/180.0f);
      goals[2].y = r*sin(210*PI/180.0f);
 
       for(int i=3; i< GOAL_SIZE; i++) {
        if(i%3==0){
          r+=0.5f;
          if(clockwise){
            goals[i].x = r*cos(210*PI/180.0f);
            goals[i].y = r*sin(210*PI/180.0f);
          }else{
            goals[i].x = 0.0f;
            goals[i].y = r;
          }
        }else if(i%3==1){
          goals[i].x = r*cos(150*PI/180.0f);
          goals[i].y = r*sin(150*PI/180.0f);
        }else{
          if(clockwise){
            goals[i].x = 0.0f;
            goals[i].y = r;
            clockwise=false;
          }else{
            goals[i].x = r*cos(210*PI/180.0f);
            goals[i].y = r*sin(210*PI/180.0f);
            clockwise=true;
          }

          
        }

      }

 /*
      for(int i= 2; i< GOAL_SIZE; i++) {
        if(i%2==0){
          goals[i].x = -0.7071*(i*0.8f);
          goals[i].y = -0.7071*(i*0.8f);
        }else{
          goals[i].x = 0.0f;
          goals[i].y = i*0.8f;
        }

      }

      //then pattern kicks in
      goals[2].x= -2.0f;
      goals[2].y= 0.0f;
      goals[3].x= -1.0f;
      goals[3].y= 1.0f;
      goals[4].x= 0.0f;
      goals[4].y= 2.0f;

      goals[5].x= 0.0f;
      goals[5].y= 3.0f;
      goals[6].x= -1.0f;
      goals[6].y= 2.0f;
      goals[7].x= -2.0f;
      goals[7].y= 1.0f;
      goals[8].x= -3.0f;
      goals[8].y= 0.0f;

      goals[9].x= -4.0f;
      goals[9].y= 0.0f;
      goals[10].x= -3.0f;
      goals[10].y= 1.0f;
      goals[11].x= -2.0f;
      goals[11].y= 2.0f;
      goals[12].x= -1.0f;
      goals[12].y= 3.0f;
      goals[13].x= 0.0f;
      goals[13].y= 4.0f;
     
      float r=  1.0f;
      int s=  1.0f;
      for(int i= 0; i< GOAL_SIZE; i++)  {
          goals[i].x= ((i/2)+1)*r*cos(((i%2== 0) ? PI/2.0f : 5.0f*PI/4.0f));
          goals[i].y= ((i/2)+1)*r*sin(((i%2== 0) ? PI/2.0f : 5.0f*PI/4.0f));
          if(isOutOfBounds(goals[i].x, goals[i].y)) {
            if(i%2== 0) {
              goals[i].x-=  s*r;
            }
            else  {
              goals[i].y+=  s*r;
              s++;
            }
          }
          //Max x value is: 7.5, Max y value is: 7.5
          goals[i].x= clamp(goals[i].x, -6.0f, 6.0f);
          goals[i].y= clamp(goals[i].y, -6.0f, 6.0f);

      }
      */

    }
    else if(publishedName=="aeneas"){ //yellow rover
          // rough start
      goals[0].x= 0.0f;
      goals[0].y= 0.0f;
      goals[1].x= 1.0f;
      goals[1].y= 1.0f;

      float r= 1.0f;
      bool clockwise=true;
      goals[2].x = 0.0f;
      goals[2].y = r;
 
       for(int i=3; i< GOAL_SIZE; i++) {
        if(i%3==0){
          r+=0.5f;
          if(clockwise){
            goals[i].x = 0.0f;
            goals[i].y = r;
          }else{
            goals[i].x = r*cos(330*PI/180.0f);
            goals[i].y = r*sin(330*PI/180.0f);
          }
        }else if(i%3==1){
          goals[i].x = r*cos(30*PI/180.0f);
          goals[i].y = r*sin(30*PI/180.0f);
        }else{
          if(clockwise){
            goals[i].x = r*cos(330*PI/180.0f);
            goals[i].y = r*sin(330*PI/180.0f);
            clockwise=false;
          }else{
            goals[i].x = 0.0f;
            goals[i].y = r;
            clockwise=true;
          }

          
        }

      }    




/*
      for(int i= 2; i< GOAL_SIZE; i++) {
        if(i%2==1){
          goals[i].x= 0.5f;  //i is used for the radius (offset to avoid edge)
          goals[i].y= i*0.8f;
        }else{
          goals[i].x= i*0.8f;  //i is used for the radius
          goals[i].y= 0.5f; // offset to avoid edge collisions
        }


      }


      //then pattern kicks in
      goals[3].x= 3.0f;
      goals[3].y= 1.0f;
      goals[4].x= 2.0f;
      goals[4].y= 2.0f;
      goals[5].x= 1.0f;
      goals[5].y= 3.0f;

      goals[6].x= 1.0f;
      goals[6].y= 4.0f;
      goals[7].x= 2.0f;
      goals[7].y= 3.0f;
      goals[8].x= 3.0f;
      goals[8].y= 2.0f;
      goals[9].x= 4.0f;
      goals[9].y= 1.0f;

      goals[10].x= 5.0f;
      goals[10].y= 1.0f;
      goals[11].x= 4.0f;
      goals[11].y= 2.0f;
      goals[12].x= 3.0f;
      goals[12].y= 3.0f;
      goals[13].x= 2.0f;
      goals[13].y= 4.0f;
      goals[14].x= 1.0f;
      goals[14].y= 5.0f;
      
      // goal locations using board coordinates
      goals[0].x = 3.0f;
      goals[0].y = 1.0f;
      goals[1].x = 3.0f;
      goals[1].y = 3.0f;
      goals[2].x = 1.0f;
      goals[2].y = 1.0f;
      
      float r=  1.0f;
      int  s= 1;
      for(int i= 0; i< GOAL_SIZE; i++)  {
          goals[i].x= ((i/2)+1)*r*cos(((i%2== 0) ? 7.0f*PI/4.0f : PI/2.0f));
          goals[i].y= ((i/2)+1)*r*sin(((i%2== 0) ? 7.0f*PI/4.0f : PI/2.0f));
          if(isOutOfBounds(goals[i].x, goals[i].y)) {
            if(i%2== 0) {
              goals[i].x+=  s*r;
            }
            else  {
              goals[i].x-=  s*r;
              s++;
            }
          }
          goals[i].x= clamp(goals[i].x, -6.0f, 6.0f);
          goals[i].y= clamp(goals[i].y, -6.0f, 6.0f);
      }
      */
    }
    else if(publishedName=="ajax"){ //white rover
      goals[0].x = 0.0f;
      goals[0].y = 0.0f;
      goals[1].x = 1.0f;
      goals[1].y = -1.0f;


      float r= 1.0f;
      bool clockwise=true;
      goals[2].x = r*cos(220*PI/180.0f);
      goals[2].y = r*sin(220*PI/180.0f);
 
       for(int i=3; i< GOAL_SIZE; i++) {
        if(i%3==0){
          r+=0.5f;
          if(clockwise){
            goals[i].x = r*cos(220*PI/180.0f);
            goals[i].y = r*sin(220*PI/180.0f);
          }else{
            goals[i].x = r*cos(320*PI/180.0f);
            goals[i].y = r*sin(320*PI/180.0f);
          }
        }else if(i%3==1){
            goals[i].x = 0.0f;
            goals[i].y = -1*r;
        }else{
          if(clockwise){
            goals[i].x = r*cos(320*PI/180.0f);
            goals[i].y = r*sin(320*PI/180.0f);
            clockwise=false;
          }else{
            goals[i].x = r*cos(220*PI/180.0f);
            goals[i].y = r*sin(220*PI/180.0f);
            clockwise=true;
          }

          
        }

      }    






      /*
      for(int i= 3; i< GOAL_SIZE; i++) {
        if(i%2==1){
          goals[i].x = -0.7071*(i*0.5f);
          goals[i].y = -0.7071*(i*0.5f);
        }else{
          goals[i].x= i*0.8f;  //i is used for the radius
          goals[i].y= -0.3f;  // to avoid edge collisions
        }


      }
      
      // waypoints for ajax

      goals[1].x = -3.0f;
      goals[1].y = -3.0f;
      goals[2].x = 0.0f;
      goals[2].y = -1.0f;
      
      float r=  1.0f;
      int   s=  1;
      for(int i= 0; i< GOAL_SIZE; i++)  {
          goals[i].x= ((i/2)+1)*r*cos(((i%2== 0) ? 5.0f*PI/4.0f : 7.0f*PI/4.0f));
          goals[i].y= ((i/2)+1)*r*sin(((i%2== 0) ? 5.0f*PI/4.0f : 7.0f*PI/4.0f));
          if(isOutOfBounds(goals[i].x, goals[i].y)) {
            if(i%2== 0) {
              goals[i].y+=  s*r;
            }
            else  {
              goals[i].x+=  s*r;
              s++;
            }
          }
          goals[i].x= clamp(goals[i].x, -6.0f, 6.0f);
          goals[i].y= clamp(goals[i].y, -6.0f, 6.0f);
      }
    */

    }
  }else{
    //is initialized
    //ROS_INFO_STREAM("Which Rover Search Called "<<name);

  }

  // goal location should be relative to board coordinates
  resultPos = nextGoal(currentLocation);

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
  //newGoalLocation.theta = oldGoalLocation.theta;
  //newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta));// (remainingGoalDist * cos(oldGoalLocation.theta));
  //newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta));// (remainingGoalDist * sin(oldGoalLocation.theta));
    
  //This makes the rover push to old location after interrupt.. basically ignoring the interrupt
  newGoalLocation.x = goals[currIndex].x;
  newGoalLocation.y = goals[currIndex].y;
  double myHeading = atan2(newGoalLocation.y - currentLocation.y, newGoalLocation.x - currentLocation.x);
  newGoalLocation.theta = myHeading;
  //ROS_INFO_STREAM_ONCE("Interrupt");

  if(name=="achilles"){ //black rover
      // This rover starts at (0,1) so subtract 1 from goalLocation.y 
    newGoalLocation.y-=1.0;
  }
  if(name=="aeneas"){ //yellow rover
    // This rover starts at (1,1) so subtract 1 from goalLocation.y and x
    newGoalLocation.x-=1.0;
    newGoalLocation.y-=1.0;
  }
  if(name=="ajax"){ //white rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    newGoalLocation.x-=1.0;
  }

  return newGoalLocation;
}
//sendClawCommand(M_PI_2);

Pos2D SearchController::nextGoal(geometry_msgs::Pose2D currentLocation){
  Pos2D goal;
  double _distance;
  /*if(!isInitiated){
    //The first point on the board to go to here:
    goal.x=goals[0].x;
    goal.y=goals[0].y;
    isInitiated=  true;
  }else*/{
    _distance = sqrt(pow(currentLocation.x-goals[currIndex].x,2.0)+pow(currentLocation.y-goals[currIndex].y,2.0));
    //ROS_INFO_STREAM("COS: published name: "<<name<<" distance "<<_distance << " to "<< currIndex);

    // go to next waypoint when goal is reached
    if(_distance<1.5){
      currIndex++;
      currIndex = currIndex%GOAL_SIZE;
      //ROS_INFO_STREAM("NEW WAYPOINT "<<currIndex << " for "<< name);
      goal.x=goals[currIndex].x;
      goal.y=goals[currIndex].y;
    }
  }

  if(name=="achilles"){ //black rover
      // This rover starts at (0,1) so subtract 1 from goalLocation.y 
    goal.y-=1.0;
  }
  if(name=="aeneas"){ //yellow rover
    // This rover starts at (1,1) so subtract 1 from goalLocation.y and x
    goal.x-=1.0;
    goal.y-=1.0;
  }
  if(name=="ajax"){ //white rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    goal.x-=1.0;
  }
  

  return goal;
}




