/*
 * 	DFNavigator.h
 * 		extends robotinoNavigator to navigate using depth first search
 * 		
 *
 * 	Developed by: Dylan Wallace
 *      Created with reference to breadth-first code from Kris Krasnosky
 * 	Organization: DASL@UNLV
 * 	Created on:   07/11/2016
 * 
 */

#ifndef PFNav__
#define PFNav__

#include "map.h"
#include "searchTreeNode.h"
#include "depthFirst.h"
#include "robotinoNavigator.h"


class DFNavigator:public robotinoNavigator{
public:
  depthFirst pathfinder;
  void navigatePath();
  float maxLinearVel;
};

#endif
