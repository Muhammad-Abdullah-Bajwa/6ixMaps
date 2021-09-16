/* 
 * File:   physics_utils.h
 * Author: Mohamed
 *
 * Description: Physics functions that use a density distribution function 
 *              to compute mass, moments and center of mass of a cuboid.
 */

#ifndef PHYSICS_UTILS_H
#define	PHYSICS_UTILS_H

#include "math_utils.h"
#include "Polynomial.h"
#include "Coordinates.h"

/*
 * Function to compute the mass of a cuboid with dimensions (xlen,ylen,zlen) and density 
 * distribution function "density function"
 */
double compute_mass_cuboid(Polynomial density_function, double xlen, double ylen, double zlen);

/*
 * Functions to compute the moment of mass in each of the x, y and z dimensions
 */
double compute_moment_x(Polynomial density_function, double xlen, double ylen, double zlen);
double compute_moment_y(Polynomial density_function, double xlen, double ylen, double zlen);
double compute_moment_z(Polynomial density_function, double xlen, double ylen, double zlen);

/*
 * Function that computes the center of mass of a cuboid with a given density distribution function
 * and dimensions (xlen,ylen,zlen)
 */
Coordinates compute_center_of_mass(Polynomial density_function, double xlen, double ylen, double zlen);

#endif	/* PHYSICS_UTILS_H */

