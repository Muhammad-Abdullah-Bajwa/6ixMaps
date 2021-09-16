/* 
 * File:   math_utils.h
 * Author: Mohamed
 *
 * Description: Mathematical utils including averaging and integrals
 */

#ifndef MATH_UTILS_H
#define	MATH_UTILS_H


#include "Polynomial.h"
#include <vector>

/*
 * Function to integrate a polynomial function "myfunc" over 3 cartesian dimensions
 * Each dimension is integrated from _start to _end
 * The number of points in each dimension specify how accurate the integration is
 */
double integrate_3d(Polynomial myfunc, double xstart, double xend, int xnum_points, 
                                       double ystart, double yend, int ynum_points, 
                                       double zstart, double zend, int znum_points);

/*
 * Helper function that averages the elements of an array with num_elements entries
 */
double average(double *my_list_to_average, int num_elements);

#endif	/* MATH_UTILS_H */

