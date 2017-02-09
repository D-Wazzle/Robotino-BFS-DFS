/*
 * 	potentialField.h
 * 		describes the basic model of a Potential Field used in potential field navigation
 * 		
 *
 * 	Developed by: Kristopher Krasnosky
 * 	Organization: DASL
 * 	Created on:   11/04/2013
 * 
 */

#ifndef __calcTools__potentialField__
#define __calcTools__potentialField__

#include <iostream>
#include <vector>
#include <cmath>


class potentialField{
public:
    // diffenet points used
    struct point{
        float x;
        float y;
    };
    struct attractivePoint{
        float x;
        float y;
        float weight;
    };
    struct repulsivePoint{
        float x;
        float y;
        float weight;
        float maxRange;
    };
    
    void addPoint(repulsivePoint p);        // adds a repulsive point to the potential field
    void addPoint(attractivePoint p);       // adds a attractive point to the potential field
    void setPoint(unsigned int index, attractivePoint p);	// set a specific point at index to a new point
    void setPoint(unsigned int index, repulsivePoint p);	// set a specific point at index to a new point
    std::vector<float> attractiveGrad(point p);		// find the gradient of the attractiveField
    std::vector<float> repulsiveGrad(point p);			// find the gradient of the repulsiveField
    std::vector<float> netGrad(point p);			// find the total gradient
    float magnitude(std::vector<float> vect);			// find the magnitued of a vector
    std::vector<float> normalize(std::vector<float> vect);	// normalize a vector
    std::vector<float> netGradNormalized(point p);		// get the normaized gradient vector IE it's direction.
    
    float distFromAttractivePoint(unsigned int i,point p);	// get the distance of point p from an index in the attractiveField
    
    void printPotentialField();				// print out the field so we can take a look at it.
    
    
private:
    double findDist(point p,repulsivePoint t);
    double findDist(point p,attractivePoint t);
    
    std::vector< attractivePoint > attractiveField;
    std::vector< repulsivePoint > repulsiveField;
};



#endif /* defined(__calcTools__potentialField__) */
