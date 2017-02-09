/*
 * 	PFNav.h
 * 		extends robotinoNavigator to navigate potential fields.
 * 		
 *
 * 	Developed by: Kristopher Krasnosky
 * 	Organization: DASL
 * 	Created on:   11/5/2013
 * 
 */

#ifndef PFNav__
#define PFNav__

#include "potentialField.h"
#include "robotinoNavigator.h"

class PFNav:public robotinoNavigator{
public:
  PFNav();
  void setPotentialField(potentialField field);	// set the internal potential field
  void spin();						// run the class.  continuously move toward lower potential
  void moveTowardTarget();				// take one step toward target
  void tileGridObstacle(float x,float y, float w, float d);
  void tileGridGoal(float x,float y, float w);
  
  potentialField navField;
  
  float movementGain;
private:

  
};


#endif  // end define