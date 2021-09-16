/* 
 * File:   physics_utils.h
 * Author: Mohamed
 *
 * Description: Computes center of mass of a cuboid with various density distribution functions.
 */

#include <iostream>
#include "math_utils.h"
#include "physics_utils.h"
#include "Polynomial.h"
#include "Coordinates.h"
#include <iomanip>

using namespace std;

int main() {
    
    //define a density polynomial
    Polynomial *my_density_function = new Polynomial(5,3,0.5,2,1,5);
    Polynomial *gx = new Polynomial(1,1,2,2,3,3);

    //find the center of mass of a cuboid of size (5,6,10) with our density function
    Coordinates center_of_mass = compute_center_of_mass(*my_density_function,5,6,10);
    Coordinates center_of_mass_gx = compute_center_of_mass(*gx,5,5,5);
    //print out center of mass
    cout << setprecision(4) << "center of mass (x,y,z) = " << center_of_mass << endl;
    cout << setprecision(4) << "center of mass (x,y,z) = " << center_of_mass_gx << endl;
    //a useless print statement
   
     delete my_density_function;
    delete gx;
    
    return 0;
}
