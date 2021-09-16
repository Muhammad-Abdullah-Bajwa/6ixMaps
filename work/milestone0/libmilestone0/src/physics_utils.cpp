#include "physics_utils.h"

//This parameter controls the accuracy of the integration
#define NUM_STEPS 10

//Use the density function to compute the mass of a cuboid
double compute_mass_cuboid(Polynomial density_function, double xlen, double ylen, double zlen)
{
    //mass is the volume integral of the density function
    double mass = integrate_3d(density_function,0,xlen,NUM_STEPS,0,ylen,NUM_STEPS,0,zlen,NUM_STEPS);
    return mass;
}

//Use density function to compute the moment x-component
double compute_moment_x(Polynomial density_function, double xlen, double ylen, double zlen)
{
    //to compute the moment we multiply the mass function by x
    density_function.multiply_by_x();
    
    //then integrate the (density_fcn * x) over the volume of the cuboid
    double moment = integrate_3d(density_function,0,xlen,NUM_STEPS,0,ylen,NUM_STEPS,0,zlen,NUM_STEPS);
    
    //finally we remove the multiplication by x
    density_function.remove_multiplicand();
    
    return moment;
}

//Use density function to compute the moment y-component
double compute_moment_y(Polynomial density_function, double xlen, double ylen, double zlen)
{
    //to compute the moment we multiply the mass function by y
    density_function.multiply_by_y();
    
    //then integrate the (density_fcn * y) over the volume of the cuboid
    double moment = integrate_3d(density_function,0,xlen,NUM_STEPS,0,ylen,NUM_STEPS,0,zlen,NUM_STEPS);
    
    //finally we remove the multiplication by y
    density_function.remove_multiplicand();
    
    return moment;
}

//Use density function to compute the moment z-component
double compute_moment_z(Polynomial density_function, double xlen, double ylen, double zlen)
{
    //to compute the moment we multiply the mass function by z
    density_function.multiply_by_z();
    
    //then integrate the (density_fcn * z) over the volume of the cuboid
    double moment = integrate_3d(density_function,0,xlen,NUM_STEPS,0,ylen,NUM_STEPS,0,zlen,NUM_STEPS);
    
    //finally we remove the multiplication by z
    density_function.remove_multiplicand();
    
    return moment;
}

//calculates the center of mass of a cuboid with a provided density distribution function
Coordinates compute_center_of_mass(Polynomial density_function, double xlen, double ylen, double zlen)
{
    //find the mass
    double mass = compute_mass_cuboid(density_function,xlen,ylen,zlen);
    
    //find x moment
    double moment_x = compute_moment_x(density_function,xlen,ylen,zlen);
    
    //find y moment
    double moment_y = compute_moment_y(density_function,xlen,ylen,zlen);

    //find z moment
    double moment_z = compute_moment_z(density_function,xlen,ylen,zlen);
    
    //find the coordinate of center of mass
    double cm_x = moment_x/mass;
    double cm_y = moment_y/mass;
    double cm_z = moment_z/mass;
    
    //define a new "Coordinates" object and pass the results to it
    Coordinates center_of_mass(cm_x,cm_y,cm_z);
    
    return center_of_mass;
    
}
