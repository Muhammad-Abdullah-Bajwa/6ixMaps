/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Test_1.cpp
 * Author: Rohit
 *
 * Created on January 28, 2018, 11:01 PM
 */

#include <cstdlib>
#include <iostream>
#include <unittest++/UnitTest++.h>

using namespace std;
 
int main(int argc, char** argv) {
    cout << "This is " << argv[0] << endl;
    //Runs all defined unit tests
    int num_failures = UnitTest::RunAllTests();
    return num_failures;
}

