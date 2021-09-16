/* 
 * File:   Polynomial.cpp
 * Author: Mohamed
 * 
 * Description: Implementation of the polynomial class
 */

#include "Polynomial.h"
#include <math.h>
#include <cassert>

enum mode { times_1, times_x, times_y, times_z };

//constructor
Polynomial::Polynomial(double c_x, int p_x, double c_y, int p_y, double c_z, int p_z) {
    coeff_x = c_x;
    coeff_y = c_y;
    coeff_z = c_z;
    pow_x = p_x;
    pow_y = p_y;
    pow_z = p_z;
    mode = times_1;
}

//return a value of the polynomial at a given (x,y,z)
double Polynomial::value(double x, double y, double z)
{
    double function_value =  coeff_x*pow(x,pow_x) + coeff_y*pow(y,pow_y)+ coeff_z*pow(z,pow_z);
    

    if(mode == times_x) {
        return function_value * x;
    } else if(mode == times_y) {
        return function_value * y;
    } else if(mode == times_z) {
        return function_value * z;
    } else {
        assert(mode == times_1);
        return function_value;
    }
}

//set the mode of the object to return the value multiplied by x
void Polynomial::multiply_by_x()
{
    mode = times_x;
}

//set the mode of the object to return the value multiplied by y
void Polynomial::multiply_by_y()
{
    mode = times_y;
}

//set the mode of the object to return the value multiplied by z
void Polynomial::multiply_by_z()
{
    mode = times_z;
}

//resets the mode of the object to return the value of the polynomial
void Polynomial::remove_multiplicand()
{
    mode = times_1;
}


Polynomial::~Polynomial() {
}

