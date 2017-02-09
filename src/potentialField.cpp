//
//  potentialField.cpp
//  calcTools
//
//  Created by Kristopher Krasnosky on 11/4/13.
//  Copyright (c) 2013 Kristopher Krasnosky. All rights reserved.
//

#include "potentialField.h"

void potentialField::addPoint(potentialField::repulsivePoint p){
    repulsiveField.push_back(p);
    return;
}

void potentialField::addPoint(potentialField::attractivePoint p){
    attractiveField.push_back(p);
    return;
}

void potentialField::setPoint(unsigned int index, potentialField::attractivePoint p){
  attractiveField[index]=p;
}

void potentialField::setPoint(unsigned int index, potentialField::repulsivePoint p){
  repulsiveField[index]=p;
}

std::vector<float> potentialField::attractiveGrad(potentialField::point p){
    std::vector<float> output(2,0);      // declare output vector and initalize with 0
    for (unsigned int i=0; i<attractiveField.size(); i++) {  // do for every element in attractive field
        output[0]=output[0]+attractiveField[i].weight * (attractiveField[i].x-p.x);     // add up all elements of attractive vector
        output[1]=output[1]+attractiveField[i].weight * (attractiveField[i].y-p.y);
    }
    return output;      // return the output
}

std::vector<float> potentialField::repulsiveGrad(potentialField::point p){
    std::vector<float> output(2,0);      // declare output vector and initalize with 0
    for (unsigned int i=0; i<repulsiveField.size(); i++){
        // if we are in the area of influence of a point.
        findDist(p, repulsiveField[i]);
        if (findDist(p, repulsiveField[i]) < repulsiveField[i].maxRange) {
            output[0]= -(output[0]+(repulsiveField[i].weight * (repulsiveField[i].x-p.x)/findDist(p, repulsiveField[i]))
                       * (1/findDist(p, repulsiveField[i]) - repulsiveField[i].maxRange));
            output[1]= -(output[1]+(repulsiveField[i].weight * (repulsiveField[i].y-p.y)/findDist(p, repulsiveField[i]))
                       * (1/findDist(p, repulsiveField[i]) - repulsiveField[i].maxRange));
        }
    }
    return output;      // return the output
}

std::vector<float> potentialField::netGrad(potentialField::point p){
  std::vector<float> output(2,0); 
  std::vector<float> repulse,attract;
  repulse=repulsiveGrad(p);
  attract=attractiveGrad(p);
  for (unsigned int i=0; i<=attractiveField.size(); i++)
    output[i]=(repulse[i]+attract[i]);
  return output;
}

float potentialField::magnitude(std::vector<float> vect){
  unsigned int i;
  float output=0;
  for (i=0; i<=vect.size(); i++){
    output=output+vect[i]*vect[i];
  }
  output=std::sqrt(output);
  return output;
}

std::vector<float> potentialField::normalize(std::vector<float> vect){
  unsigned int i;
  std::vector<float> output(vect.size(),0);
  float mag=magnitude(vect);
  for (i=0; i<=vect.size(); i++){
    output[i]=vect[i]/mag;
  }
  return output;
}

std::vector<float> potentialField::netGradNormalized(potentialField::point p){
  return normalize(netGrad(p));
}

float potentialField::distFromAttractivePoint(unsigned int i,potentialField::point p){
  std::vector<float> vect(2,0);
  vect[0]=p.x-attractiveField[i].x;
  vect[1]=p.y-attractiveField[i].y;
  return magnitude(vect);
}

void potentialField::printPotentialField(){
    std::cout << "attractive points" << std::endl;
    for (unsigned int i=0; i<attractiveField.size(); i++) {
        std::cout << attractiveField[i].x << "," << attractiveField[i].y <<","<<attractiveField[i].weight;
        std::cout << ":";
    }
    std::cout << std::endl << "repulsive points" << std::endl;
    for (unsigned int i=0; i<repulsiveField.size(); i++) {
        std::cout << repulsiveField[i].x << "," << repulsiveField[i].y<<","<<repulsiveField[i].weight<<","<<repulsiveField[i].maxRange;
        std::cout << ":";
    }
    std::cout << std::endl;
    return;
}


/*
 *  private funcitons
 *
 */


double potentialField::findDist(potentialField::point p, potentialField::repulsivePoint t){
    double x=(p.x-t.x);
    double y=(p.y-t.y);
    return std::sqrt(x*x + y*y);
}

double potentialField::findDist(potentialField::point p, potentialField::attractivePoint t){
    double x=(p.x-t.x);
    double y=(p.y-t.y);
    return std::sqrt(x*x + y*y);
}






