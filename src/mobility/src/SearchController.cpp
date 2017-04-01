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
Pos2D roverWorldCoordAdjust(Pos2D arg, std::string argName){
  //takes rover coords and returns world coords
  if(argName=="achilles"){ //black rover
    // This rover starts at (0,1) so subtract 1 from goalLocation.y 
    arg.y-=1.0;
  }
  if(argName=="aeneas"){ //yellow rover
    // This rover starts at (1,1) so subtract 1 from goalLocation.y and x
    arg.x-=1.0;
    arg.y-=1.0;
  }
  if(argName=="ajax"){ //white rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    arg.x-=1.0;
  }
  if(argName=="diomedes"){ //red rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    arg.x+= 1;
  }
  if(argName=="hector"){ //blue rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    arg.y+= 1;
  }
  if(argName=="paris"){ //orange rover
    // This rover starts at (1,0) so subtract 1 from goalLocation.x
    arg.x+= 1;
    arg.y+= 1;
  }

  return arg;
}

//constructor
SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();

  isInitiated=  false;
  currIndex = 0;
}

/**
 * This code implements COS HEX Sweep Search
 * Search Method
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation, std::string publishedName) {
  //Pos2D is a simple (x,y) stuct found in the header file
  Pos2D resultPos;
  // Pose2D is a ROS struct (x,y,heading)
  geometry_msgs::Pose2D goalLocation;

  if(!isInitiated)  {
    isInitiated=  true;
    // name is a property of the searchController and holds the rover name
    // this will be passed into search method until we can learn ROS to get it.
    name = publishedName;

    // Sets Waypoint array once
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
        //Max x value is: 7.5, Max y value is: 7.5
        goals[i].x= clamp(goals[i].x, -6.0f, 6.0f);
        goals[i].y= clamp(goals[i].y, -6.5f, 6.5f);

      }

    }
    else if(publishedName=="aeneas"){ //yellow rover
      
      // rough start
      goals[0].x= 0.0f;
      goals[0].y= 0.0f;
      goals[1].x= 2.0f;
      goals[1].y= 2.0f;

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
        //Max x value is: 7.5, Max y value is: 7.5
        goals[i].x= clamp(goals[i].x, -6.0f, 6.0f);
        goals[i].y= clamp(goals[i].y, -6.5f, 6.5f);

      }    

    }
    else if(publishedName=="ajax"){ //white rover
      goals[0].x = 0.0f;
      goals[0].y = 0.0f;
      goals[1].x = 1.0f;
      goals[1].y = -2.0f;


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
        //Max x value is: 7.5, Max y value is: 7.5
        goals[i].x= clamp(goals[i].x, -6.0f, 6.0f);
        goals[i].y= clamp(goals[i].y, -6.5f, 6.5f);

      }    

    }
    else if(publishedName=="diomedes"){ //Red rover
      
      // set waypoints with respect to the board origin
      // rough start to try to avoid other rovers at start
      goals[0].x= 0;
      goals[0].y= 9;
      goals[1].x= -9;
      goals[1].y= 9;
      goals[2].x= -9;
      goals[2].y= 0;
      goals[3].x= -9;
      goals[3].y= -9;
      goals[4].x= -8.5f;
      goals[4].y= -9;
      goals[5].x= -8.5f;
      goals[5].y= 0;
      goals[6].x= -8.5f;
      goals[6].y= 8.5f;
      goals[7].x= 0;
      goals[7].y= 8.5f;
      goals[8].x= 0;
      goals[8].y= 7;
      goals[9].x= -7;
      goals[9].y= 7;
      goals[10].x= -7;
      goals[10].y= -7;
      goals[11].x= -5;
      goals[11].y= -7;
      goals[12].x= -5;
      goals[12].y= 6;
      goals[13].x= -9;
      goals[13].y= 9;
 /*
      goals[0].x = 0;
      goals[0].y = 9.5f;
      goals[1].x= -9.5f;
      goals[1].y= 9.5f;

      float r= 0.5f;
      bool clockwise=true;
      int skip= 0;
       for(int i=3; i< GOAL_SIZE; i++) {
         if(skip== 0) {
           if(clockwise)  {
             goals[i].x=  (9.5f-r)*cos(220*PI/180.0f);
             goals[i].y=  (9.5f-r)*sin(220*PI/180.0f);
             r+=  0.5f;
           }
           else {
             goals[i].x=  0.0f;
             goals[i].y=  9.5f-r;
             r+=  0.5f;
           }
           skip++;
         }

         if(skip== 1) {
           if(clockwise)  {
             goals[i].x=  (9.5f-r)*cos(220*PI/180.0f);
             goals[i].y=  (9.5f-r)*sin(220*PI/180.0f);
             clockwise= false;
           }
           else {
             goals[i].x=  0.0f;
             goals[i].y=  9.5f-r;
             clockwise= true;
           }
           skip++;
         }
         if(skip== 2)  { // Corners
            goals[i].x= -9.5f-r;
            goals[i].y= 9.5f-r;
            skip= 0;
         }
      }
*/
    }
    // Sets Waypoint array once
    if(publishedName=="hector"){ //blue rover
      float _cos= cos(315*PI/180.0f);
      float _sin= sin(315*PI/180.0f);

      goals[0].x= 9.5f*_cos;
      goals[0].y= 9.5f*_sin;
      goals[1].x= 9.5f;
      goals[1].y= 9.5f;
      goals[2].x= 0;
      goals[2].y= 9.5f;
      goals[3].x= 0;
      goals[3].y= 9;
      goals[4].x= 9;
      goals[4].y= 9;
      goals[5].x= 9*_cos;
      goals[5].y= 9*_sin;
      goals[6].x= 8.5f*_cos;
      goals[6].y= 8.5f*_sin;
      goals[7].x= 8.5f;
      goals[7].y= 8.5f;
      goals[8].x= 0;
      goals[8].y= 8.5f;
      goals[9].x= 0;
      goals[9].y= 8;
      goals[10].x= 8;
      goals[10].y= 8;
      goals[11].x= 8*_cos;
      goals[11].y= 8*_sin;

      /*
      goals[0].x= 9.5f*cos(315*PI/180.0f);
      goals[0].y= 9.5f*sin(315*PI/180.0f);
      goals[1].x= 9.5f;
      goals[1].y= 9.5f;

      float r= 0.5f;
      bool clockwise=true;
      int skip= 0;*/
      /*
       for(int i=3; i< GOAL_SIZE; i++) {
         if(skip== 0) {
           if(clockwise)  {
             goals[i].x=  0.0f;
             goals[i].y=  9.5f-r;
             r+=  0.5f;
           }
           else {
             goals[i].x=  (9.5f-r)*cos(315*PI/180.0f);
             goals[i].y=  (9.5f-r)*sin(315*PI/180.0f);
             r+=  0.5f;
           }
             skip++;
         }

         if(skip== 1) {
           if(clockwise)  {
             goals[i].x=  0.0f;
             goals[i].y=  9.5f-r;
             clockwise= false;
           }
           else {
             goals[i].x=  (9.5f-r)*cos(315*PI/180.0f);
             goals[i].y=  (9.5f-r)*sin(315*PI/180.0f);
             clockwise=true;
           }
           skip++;
         }
         if(skip== 2)  { // Corners
            goals[i].x= 9.5f-r;
            goals[i].y= 9.5f-r;
            skip= 0;
         }
      }
      */
    }
    if(publishedName=="paris"){ //orange rover
      bool  clockwise=  true;
      int r=  1;
      float theta= tan(220*PI/180.0f);
      float phi=  tan(315*PI/180.0f);

      for(int i= 0; i< GOAL_SIZE; i++)  {
        if(clockwise) {
          goals[i].x= (-9.5f+0.5f*r)/theta;
          goals[i].y= (-9.5f+0.5f*r);
          i++;
          goals[i].x= (-9.5f+0.5f*r)/phi;
          goals[i].y= (-9.5f+0.5f*r);
          clockwise=  false;
        }
        else  {
          goals[i].x= (-9.5f+0.5f*r)/phi;
          goals[i].y= (-9.5f+0.5f*r);
          i++;
          goals[i].x= (-9.5f+0.5f*r)/theta;
          goals[i].y= (-9.5f+0.5f*r);
          clockwise=  true;
        }
      }
      /*
      // set waypoints with respect to the board origin
      // rough start to try to avoid other rovers at start
      goals[0].x= 9.5f*cos(220*PI/180.0f);
      goals[0].y= 9.5f*sin(220*PI/180.0f);
      goals[1].x= 9.5f*cos(315*PI/180.0f);
      goals[1].y= 9.5f*sin(315*PI/180.0f);

      float r= 0.5f;
      bool clockwise=false;
      bool  skip= false;
 
       for(int i=3; i< GOAL_SIZE; i++) {
         if(clockwise)  {
           if(!skip) {
             goals[i].x=  (9.5f-r)*cos(315*PI/180.0f);
             goals[i].y=  (9.5f-r)*cos(315*PI/180.0f);
             r+=  0.5f;
             skip=  true;
           }
           else {
             goals[i].x=  (9.5f-r)*cos(315*PI/180.0f);
             goals[i].y=  (9.5f-r)*cos(315*PI/180.0f);
             skip=  false;
             clockwise= false;
           }
         }
         else {
           if(!skip)  {
            goals[i].x= (9.5f-r)*cos(220*PI/180.0f);
            goals[i].y= (9.5f-r)*sin(220*PI/180.0f);
            r+= 0.5f;
            skip= true;
           }
           else {
            goals[i].x= (9.5f-r)*cos(220*PI/180.0f);
            goals[i].y= (9.5f-r)*sin(220*PI/180.0f);
            skip= false;
            clockwise=  true;
           }
         }
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
  //ROS_INFO_STREAM_ONCE("Continue Search After Interrupt");
  geometry_msgs::Pose2D newGoalLocation;

  Pos2D resultPos;
  resultPos.x = goals[currIndex].x;
  resultPos.y = goals[currIndex].y;
  resultPos = roverWorldCoordAdjust(resultPos, name);

  //This makes the rover push to old location after interrupt... basically ignoring the interrupt
  newGoalLocation.x = resultPos.x;
  newGoalLocation.y = resultPos.y;
  double myHeading = atan2(newGoalLocation.y - currentLocation.y, newGoalLocation.x - currentLocation.x);
  newGoalLocation.theta = myHeading;
  
  return newGoalLocation;
}


// COS nextGoal Method iterates through the waypoints
Pos2D SearchController::nextGoal(geometry_msgs::Pose2D currentLocation){
  Pos2D goal;
  double _distance;
      
  //cycle index back to one to repeat path
  if(name== "diomedes") {
    currIndex%= 14;
  }
  else if(name== "hector")  {
    currIndex%= 12;
  }
  else  {
    currIndex = currIndex%GOAL_SIZE;
  }
  if(fabs(goals[currIndex].y)>6.0f){
      currIndex=0;
  }
  _distance = sqrt(pow(currentLocation.x-goals[currIndex].x,2.0)+pow(currentLocation.y-goals[currIndex].y,2.0));
  //ROS_INFO_STREAM("COS: published name: "<<name<<" distance "<<_distance << " to "<< currIndex);

  // go to next waypoint when goal is reached
  if(_distance<1.5){
    currIndex++;
    //currIndex = currIndex%GOAL_SIZE;
    //ROS_INFO_STREAM("NEW WAYPOINT "<<currIndex << " for "<< name);
    goal.x=goals[currIndex].x;
    goal.y=goals[currIndex].y;
  }

  Pos2D adjustedGoal = roverWorldCoordAdjust(goal, name);
  goal.x = adjustedGoal.x;
  goal.y = adjustedGoal.y;

  return goal;
}




