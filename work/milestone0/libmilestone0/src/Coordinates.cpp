/* 
 * File:   Coordinates.cpp
 * Author: Mohamed
 * 
 * Description: Implementation of the Coordinates class.
 */

#include "Coordinates.h"

//constructor
Coordinates::Coordinates(double x_in, double y_in, double z_in) {
    x = x_in;
    y = y_in;
    z = z_in;
}

//overloading the "<<" operator allows us to change what 
//gets printed when we print an instance of this object
std::ostream& operator<<(std::ostream &strm, const Coordinates &c) {
  return strm << "(" << c.x << "," << c.y << "," << c.z << ")";
}

Coordinates::~Coordinates() {
}

