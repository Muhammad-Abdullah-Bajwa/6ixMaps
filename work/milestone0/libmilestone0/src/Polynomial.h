/* 
 * File:   Polynomial.h
 * Author: Mohamed
 *
 * Description: Object that defines a polynomial of up to 3 dimensions
 */

#ifndef POLYNOMIAL_H
#define	POLYNOMIAL_H

class Polynomial {
    double coeff_x;
    int pow_x;
    double coeff_y;
    int pow_y;
    double coeff_z;
    int pow_z;
    int mode;
public:
    Polynomial(double, int, double, int, double, int);
    double value(double, double, double);
    void multiply_by_x();
    void multiply_by_y();
    void multiply_by_z();
    void remove_multiplicand();
    virtual ~Polynomial();
};

#endif	/* POLYNOMIAL_H */

