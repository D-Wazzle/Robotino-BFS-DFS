//
//  map.h
//  searchTree
//
//  Created by Kristopher Krasnosky on 11/20/13.
//  Copyright (c) 2013 DASL. All rights reserved.
//

#ifndef __searchTree__map__
#define __searchTree__map__

#include <iostream>
#include <vector>

class map{
public:
    map();
    struct gridPoint{
        bool isObstacle;
        bool isExplored;
        bool isGoal;
        int location[2];
    };
    std::vector<std::vector<gridPoint> > grid;
    void createGrid(int xDim,int yDim);
    void setOriginOnGrid(int x, int y);
    gridPoint *getPoint(int x,int y);
    void addGoal(int x,int y);
    void addObstacle(int x,int y);
    float scale;
    int gridDim[2];
private:
    int originLocation[2];
    
    gridPoint * border;
    
};

#endif /* defined(__searchTree__map__) */
