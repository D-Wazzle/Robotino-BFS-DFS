//
//  breadthFirst.h
//  searchTree
//
//  Created by Kristopher Krasnosky on 11/20/13.
//  Copyright (c) 2013 DASL. All rights reserved.
//

#ifndef __searchTree__breadthFirst__
#define __searchTree__breadthFirst__

#include <iostream>
#include <vector>
#include "searchTreeNode.h"

class breadthFirst{
public:
    breadthFirst();
    //~breadthFirst();
    std::vector<std::vector<int> > findPath(searchTreeNode *startNode);   // run repeatedly to find path
    void runQueue();	// runs the next point in the queue
    void start(int x,int y);  // used to start the search from point x,y
    map theMap;		// this is where the map data is stored access by mybreadthFirst.map.someMemberFunction();
    std::vector<std::vector<int> > goHome(searchTreeNode *node);  // after the location is found we can follow the parent node tree to the start location and retrieve our path
    std::vector<std::vector<int> > path;  // where we actually store the path
    float getMapScale();	//  gets the scale of the map
    void setMapScale(float s);
    map::gridPoint * debug;
private:
    
    std::vector<searchTreeNode *> queue;
    bool checkPoint(int x,int y);
    searchTreeNode *goalNode;
    bool goalFound;
};

#endif /* defined(__searchTree__breadthFirst__) */
