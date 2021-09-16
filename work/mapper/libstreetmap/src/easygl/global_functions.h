/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   global_functions.h
 * Author: aroraro7
 *
 * Created on February 21, 2018, 6:01 PM
 */
#pragma once
#ifndef GLOBAL_FUNCTIONS_H
#define GLOBAL_FUNCTIONS_H

#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <time.h>
#include <pthread.h>
#include <random>



//This object contains all the information of a node for dijkstras
struct Node_Edge_Object{
    IntersectionIndex id;
    double weight;
    StreetSegmentIndex SegId;
};

//defining name-spaces and making typedefs for long type names
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;
typedef bgi::rtree<value, bgi::linear < 16 >> rtree;



//Functions to communicate throughout the program, storing and transferring objects
void last_map_name(std::string);
void search_bar_search (std::string s);


//Needed to make this global for access in other files
void draw_buttons();

//Calculates and returns the determinant of the
double determinant(std::pair<LatLon,LatLon> begin, std::pair<LatLon,LatLon> end);

//Calculates the x value from a LATLON
double lon_to_x(LatLon t);
//Calculates the y value from a LATLON
double lat_to_y(LatLon t);



//For each element, if 0 straight, 1 right turn, -1 left turn
std::vector<std::string> all_turns(std::vector<unsigned> path);
std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start,
        std::vector<unsigned> targets,
        const double turn_penalty);


//External variables
extern bool opened_found_path; //The path was found
extern bool has_opened; //The menu is opened
extern double min_lat; //minimum latitude
extern double max_lat; //maximum latitude
extern double min_lon; //minimum longitude
extern double max_lon; //maximum longitude
extern double average_lat_again; //average latitude calculated by mode
extern int MWIDTH; //menu width
extern int max_segments; //total number of streets in a path
extern int path_at; //current street on path showing


extern rtree POI_RTree; //rtree carrying the pois
extern rtree f_rtree; //rtree carrying the features
extern rtree segments_RTree; //rtree carrying the segments
extern std::vector<std::string> directions; // stores all the directions
extern std::vector<std::deque < Node_Edge_Object>> adjacency_list;
//These are for the keyboard input for the search bar
extern std::unordered_map<std::string, std::vector<unsigned>> POI_list;
extern std::unordered_map<int,char> keypress_LUT;
extern std::unordered_map<std::string, std::vector<unsigned>> streetID_from_streetName;
//carries the directions along a given path
extern std::vector<std::string> directions;
extern bool keep_message; //need for keeping a message in the display menu
extern unsigned closest_POI; //id of the closest poi found from closest poi search

/**/
\
#endif /* GLOBAL_FUNCTIONS_H */

