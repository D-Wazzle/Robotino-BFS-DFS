//
//  map.cpp
//  searchTree
//
//  Created by Kristopher Krasnosky on 11/20/13.
//  Copyright (c) 2013 DASL. All rights reserved.
//

#include "map.h"

map::map(){
    originLocation[0]=0;
    originLocation[1]=0;
    scale=0.2794;  // this scale will convert the units to floor tile grid.  ~1ft
    border=new gridPoint;
    border->isExplored=true;
    border->isObstacle=true;
}

void map::createGrid(int xDim, int yDim){
    gridPoint p {false,false,false};
    std::vector<gridPoint> yInit;
    for(int i=0;i < yDim; i++){
      p.location[1]=i;
      yInit.push_back(p);
    }
    for(int i=0 ; i<xDim ;  i++){
      for(int j=0;j < yDim; j++){
	yInit[j].location[0]=i;
	grid.push_back(yInit);
      }
    }
    gridDim[0]=xDim;
    gridDim[1]=yDim;
}

void map::setOriginOnGrid(int x, int y){
    originLocation[0]=x;
    originLocation[1]=y;
    for(unsigned int i=0;i<grid.size();i++){
        for(unsigned int j=0;j<grid[i].size();j++){
            grid[i][j].location[0]=i-originLocation[0];
            grid[i][j].location[1]=j-originLocation[1];
        }
    }
}

map::gridPoint *map::getPoint(int x,int y){
    //std::cout<<x+originLocation[0]<<","<<y+originLocation[1]<< std::endl;
    
    if(x+originLocation[0]<0||x+originLocation[0]>gridDim[0]-1){
      //std::cout<< " returned border 1"  <<std::endl;  
      return border;
	
    }
    
    if(y+originLocation[1]<0||x+originLocation[1]>gridDim[1]-1){
      //std::cout<< " returned border 2"  <<std::endl;  
      return border;
    }
    else{
      //std::cout<< " returned interior "  <<std::endl;
      return &grid[x+originLocation[0]][y+originLocation[1]];
      
    }
}

void map::addGoal(int x, int y){
    getPoint(x, y)->isGoal=true;
}

void map::addObstacle(int x, int y){
    getPoint(x, y)->isObstacle=true;
}