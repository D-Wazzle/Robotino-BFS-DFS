//
//  breadthFirst.cpp
//  searchTree
//
//  Created by Kristopher Krasnosky on 11/20/13.
//  Copyright (c) 2013 DASL. All rights reserved.
//

#include "breadthFirst.h"

breadthFirst::breadthFirst(){
    goalFound=true;
}


// expands a node creates childern in viable map spaces and adds them to the queue
std::vector<std::vector<int> > breadthFirst::findPath(searchTreeNode * startNode){
    
  
    std::vector<std::vector<int> > output;
    int x=startNode->point->location[0];
    int y=startNode->point->location[1];
    
    // check every possible direction
      for(int i=-1;i<=1;i++){
        for (int j=-1; j<=1; j++) {
            if(i==0 && j==0);
            else{
                
                if(checkPoint(x+i,y+j)){  // see if our current working point is viable for a node.
                    searchTreeNode *temp;  // creates a new node
                    temp=new searchTreeNode;  // allocate its memory
                    temp->parent.push_back(startNode);   // add our start node to the list of parents
                    temp->point=theMap.getPoint(x+i, y+j);  // set temps point to the map point it is for.
                    queue.push_back(temp);  // put it in the queue.
                    startNode->children.push_back(temp);  // add temp as a child of startNode
                    temp->point->isExplored=true;
                    if (temp->point->isGoal==true) {
                        goalNode=temp;
                        goalFound=true;
                        path=goHome(temp);
                        output=path;
                    }
                }
                
            }
        }
      }
    startNode->point->isExplored=true;  // say we explored the start node
    
    return output;
}


void breadthFirst::runQueue(){
    while(queue.size()>0){  // while there is stuff in the queue
	searchTreeNode * temp = queue[0];
        findPath(queue[0]);         // operate on the first queue
        queue.erase(queue.begin()); // after operated delete it
    }
}

void breadthFirst::start(int x, int y){
    searchTreeNode *node= new searchTreeNode;
    
    node->point=theMap.getPoint(x,y);
    
    queue.push_back(node);
    
    runQueue();
   
    return;
}

std::vector<std::vector<int> > breadthFirst::goHome(searchTreeNode *node){
    std::vector<std::vector<int> > output;
    
    while(node->parent.size()>0){
        std::vector<int> temp;
        temp.push_back(node->point->location[0]);
        temp.push_back(node->point->location[1]);
        output.insert(output.begin(), temp);
        node=node->parent[0];
    }
    std::vector<int> temp;
    temp.push_back(node->point->location[0]);
    temp.push_back(node->point->location[1]);
    output.insert(output.begin(), temp);

    
    return output;
}

float breadthFirst::getMapScale(){
  return theMap.scale;
}

void breadthFirst::setMapScale(float s){
  theMap.scale=s;
  return;
}


// checks to see if a map location is viable to place a node in.
bool breadthFirst::checkPoint(int x, int y){
  
    
    debug=theMap.getPoint(x, y);
    if(theMap.getPoint(x, y)==NULL) return false;
    else if(theMap.getPoint(x, y)->isExplored) return false;
    else if(theMap.getPoint(x, y)->isObstacle) return false;
    else{
        return true;
    }
}
