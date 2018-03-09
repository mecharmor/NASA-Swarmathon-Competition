#include "SearchController.h"
#include <angles/angles.h>
#include <cmath>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
double PI = 3.14159;

if(roverName=="achilles"){ //black rover
    result.type = waypoint;

    Point  searchLocation;

    // setting int waypoint
    searchLocation.theta = currentLocation.theta;
    searchLocation.x = 0;
    searchLocation.y = 0;

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    
    searchLocation.x = -2;
    searchLocation.y = 1;
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    float r= 1.0f;
      bool clockwise=true;
      searchLocation.x = r*cos(210*PI/180.0f);
      searchLocation.y = r*sin(210*PI/180.0f);
      result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

       for(int i=3; i< 10; i++) {
        if(i%3==0){
          r+=0.5f;
          if(clockwise){
            searchLocation.x = r*cos(210*PI/180.0f);
            searchLocation.y = r*sin(210*PI/180.0f);
          }else{
            searchLocation.x = 0.0f;
            searchLocation.y = r;
          }
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

        }else if(i%3==1){
          searchLocation.x = r*cos(150*PI/180.0f);
          searchLocation.y = r*sin(150*PI/180.0f);
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

        }else{
          if(clockwise){
            searchLocation.x = 0.0f;
            searchLocation.y = r;
            clockwise=false;
          }else{
            searchLocation.x = r*cos(210*PI/180.0f);
            searchLocation.y = r*sin(210*PI/180.0f);
            clockwise=true;
          }
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);


        }
        //Max x value is: 7.5, Max y value is: 7.5
        //searchLocation.x= clamp(searchLocation.x, -6.0f, 6.0f);
        //searchLocation.y= clamp(searchLocation.y, -6.5f, 6.5f);
        //result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
       }


    for(int i = 0; i < result.wpts.waypoints.size();i++){
      result.wpts.waypoints.at(i).x += 1.2;
    }
       
} else if ( roverName == "ajax"){ //white rover
    result.type = waypoint;

    Point  searchLocation;

    int val = 2;
    for(int i = 0; i < 6; i++){
      searchLocation.y = 2 - (i * 0.5);
     
      if(i%2 == 1){
        val += 2;
      }
      searchLocation.y = val * pow(-1, i);
     result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    }
     for(int i = 0; i < result.wpts.waypoints.size();i++){
      result.wpts.waypoints.at(i).x -= 1.2;
    }

} else if ( roverName == "aeneas"){ //yellow rover
    
    result.type = waypoint;

    Point  searchLocation;

     int val = 2;
    for(int i = 0; i < 6; i++){
      searchLocation.y = -2 - (i * 0.5);
     
      if(i%2 == 1){
        val += 2;
      }
      searchLocation.x = val * pow(-1, i);
     result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    }
     for(int i = 0; i < result.wpts.waypoints.size();i++){
      result.wpts.waypoints.at(i).y += 1.2;
    }
}else if (roverName == "paris"){ //orange rover

}else if (roverName == "hector"){ //blue rover
  
}else if (roverName == "diomedes"){ //red rover
  
}
return result;


/*
  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    
    
    return result;
  }
*/
}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetRoverName(string name){
  roverName = name;
  cout << roverName << endl;
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


