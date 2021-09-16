/* 
 * File:   Coordinates.h
 * Author: Mohamed
 *
 * Description: Object that stores (x,y,z) coordinates
 */

#ifndef COORDINATES_H
#define	COORDINATES_H

#include <iostream>

class Coordinates {
    double x;
    double y;
    double z;
public:
    Coordinates(double,double,double);
    virtual ~Coordinates();
private:
    //overloading the standard "<<" operator ad giving it "friend" access 
    //allows it to access the private variables in this class
    friend std::ostream& operator<<(std::ostream &strm, const Coordinates &c);
};

#endif	/* COORDINATES_H */

