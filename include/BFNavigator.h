/*
 * 	BFNavigator.h
 * 		extends robotinoNavigator to navigate using breadth frist search
 * 		
 *
 * 	Developed by: Kristopher Krasnosky
 * 	Organization: DASL
 * 	Created on:   11/20/2013
 * 
 */

#ifndef PFNav__
#define PFNav__

#include "map.h"
#include "searchTreeNode.h"
#include "breadthFirst.h"
#include "robotinoNavigator.h"


class BFNavigator:public robotinoNavigator{
public:
  breadthFirst pathfinder;
  void navigatePath();
  float maxLinearVel;
};

#endif
