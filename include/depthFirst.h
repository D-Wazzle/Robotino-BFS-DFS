//
//  depthFirst.h
//  searchTree
//
//  Created by Dylan Wallace on 07/11/16.
//  Using Breadth-First code from Kris Krasnosky as reference.
//  Copyright (c) 2016 DASL@UNLV. All rights reserved.
//

#ifndef __searchTree__depthFirst__
#define __searchTree__depthFirst__

#include <iostream>
#include <vector>
#include "searchTreeNode.h"

class depthFirst{
public:
    depthFirst();
    //~depthFirst();
    std::vector<std::vector<int> > findPath(searchTreeNode *startNode);   // run repeatedly to find path
    void runStack();	// runs the next point in the stack
    void start(int x,int y);  // used to start the search from point x,y
    map theMap;		// this is where the map data is stored access by mydepthFirst.map.someMemberFunction();
    std::vector<std::vector<int> > goHome(searchTreeNode *node);  // after the location is found we can follow the parent node tree to the start location and retrieve our path
    std::vector<std::vector<int> > path;  // where we actually store the path
    float getMapScale();	//  gets the scale of the map
    void setMapScale(float s);
    map::gridPoint * debug;
private:
    
    std::vector<searchTreeNode *> stack;
    bool checkPoint(int x,int y);
    searchTreeNode *goalNode;
    bool goalFound;
};

#endif /* defined(__searchTree__depthFirst__) */
