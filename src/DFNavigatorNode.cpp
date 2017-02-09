//
//  DFNavigatorNode.cpp
//  searchTree
//
//  Created by Dylan Wallace on 07/11/16.
//  Using Breadth-First code from Kris Krasnosky as reference.  
//  Copyright (c) 2016 DASL@UNLV. All rights reserved.
//

#include <iostream>
#include "DFNavigator.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "DFNavigatorNode");
   
    map myMap;
    depthFirst mySearch;
    DFNavigator navigator;
    
    
    myMap.createGrid(12, 12);
    
    myMap.setOriginOnGrid(0, 0);

    myMap.addGoal(0, 7);
    
    for(int i=0;i<=4;i++){
      myMap.addObstacle(i, 3);
    }
    for(int i=0;i<=4;i++){
      myMap.addObstacle(i, 6);
    }
    for(int i=0;i<=4;i++){
      myMap.addObstacle(8, i);
    }
    for(int i=7;i<=9;i++){
      myMap.addObstacle(4, i);
    }
    for(int i=7;i<=9;i++){
      myMap.addObstacle(6, i);
    }
    myMap.addObstacle(5, 7);
    myMap.addObstacle(5, 9);
    myMap.addObstacle(8, 8);
    myMap.addObstacle(11, 11);
    
      
    mySearch.theMap=myMap;
    
    mySearch.start(2, 1);

    for (int i=0; i<mySearch.path.size(); i++) {
        std::cout << "{" << mySearch.path[i][0] << "," << mySearch.path[i][1] << "},";
    }  
    navigator.setOdom(0,0,0);
    navigator.pathfinder.theMap=myMap;
    navigator.navigatePath();

    std::cout<<std::endl;
    return 0;
}

