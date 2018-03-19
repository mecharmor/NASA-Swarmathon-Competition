#include "SearchController.h"
#include <angles/angles.h>

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
    result.type = waypoint; 
    Point  searchLocation; 
    float angleOfRotation = 0.261799;
    double forwardStep = 0.5;

    if (first_waypoint)
    {
      // First spawn do a 180 and move out 1.5m
      first_waypoint = false; 
      searchLocation.x = currentLocation.x + 1.5*cos(currentLocation.theta + M_PI);
      searchLocation.y = currentLocation.y + 1.5*sin(currentLocation.theta + M_PI);
    }
    else
    {

      if (currentLocation.x >= 0 && currentLocation.y >= 0){
        // If In Q1 go to Q2
        cout << "In Q1" << endl;
        searchLocation.x = currentLocation.x - abs(forwardStep * cos(currentLocation.theta + angleOfRotation));
        searchLocation.y = currentLocation.y + abs(forwardStep * sin(currentLocation.theta + angleOfRotation));

      } else if (currentLocation.x <= 0 && currentLocation.y > 0){
        // If In Q2 go to Q3
        cout << "In Q2" << endl;
        searchLocation.x = currentLocation.x - abs(forwardStep * cos(currentLocation.theta + angleOfRotation));
        searchLocation.y = currentLocation.y - abs(forwardStep * sin(currentLocation.theta + angleOfRotation));

      } else if (currentLocation.x <= 0 && currentLocation.y <= 0){
        // If In Q3 go to Q4
        cout << "In Q3" << endl;
        searchLocation.x = currentLocation.x + abs(forwardStep * cos(currentLocation.theta + angleOfRotation));
        searchLocation.y = currentLocation.y - abs(forwardStep * sin(currentLocation.theta + angleOfRotation));

      } else if (currentLocation.x >= 0 && currentLocation.y <= 0){
        // If In Q4 go to Q1
        cout << "In Q4" << endl;
        searchLocation.x = currentLocation.x + abs(forwardStep * cos(currentLocation.theta + angleOfRotation));
        searchLocation.y = currentLocation.y + abs(forwardStep * sin(currentLocation.theta + angleOfRotation));
      }
    }
    cout << centerLocation.x << endl;
    cout << centerLocation.y << endl;

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation); 
    return result;
  

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


