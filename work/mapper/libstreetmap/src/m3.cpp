/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#define M_TO_KM 1000
#define H_TO_S 3600
#include <vector>
#include <string>
#include "LatLon.h"
#include <math.h>
#include "StreetsDatabaseAPI.h"
#include <string>
#include <iostream>
#include <iterator>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <X11/Xlib.h>
#include "graphics.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include "global_functions.h"
#include "m2.h"
#include "m1.h"
#include "m3.h"
#include <sstream>
#include <chrono>
#include <ctime>



unsigned closest_POI;


// Returns the time required to travel along the path specified, in seconds. 
// The path is given as a vector of street segment ids, and this function 
// can assume the vector either forms a legal path or has size == 0.
// The travel time is the sum of the length/speed-limit of each street 
// segment, plus the given turn_penalty (in seconds) per turn implied by the path. 
// A turn occurs when two consecutive street segments have different street IDs.

double compute_path_travel_time(const std::vector<unsigned>& path,
        const double turn_penalty) {

    if (path.begin() == path.end())
        return 0;
    double pathTime = 0;

    for (unsigned i = 0; i < path.size(); ++i) {

        double tempLength = find_street_segment_length(path[i]);
        double tempLimit = getStreetSegmentInfo(path[i]).speedLimit;

        if (i == 0) {
            pathTime = pathTime + (tempLength * 1 / M_TO_KM * 1 / tempLimit) * H_TO_S;
        } else {
            pathTime = pathTime + (tempLength * 1 / M_TO_KM * 1 / tempLimit) * H_TO_S;
            if (getStreetSegmentInfo(path[i - 1]).streetID != getStreetSegmentInfo(path[i]).streetID) {
                pathTime = pathTime + turn_penalty;
            }
        }

    }
    return pathTime;
}

// Returns a path (route) between the start intersection and the end 
// intersection, if one exists. This routine should return the shortest path
// between the given intersections when the time penalty to turn (change
// street IDs) is given by turn_penalty (in seconds).
// If no path exists, this routine returns an empty (size == 0) vector. 
// If more than one path exists, the path with the shortest travel time is 
// returned. The path is returned as a vector of street segment ids; traversing 
// these street segments, in the returned order, would take one from the start 
// to the end intersection.

std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start,
        const unsigned intersect_id_end,
        const double turn_penalty) {

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<unsigned> path;
    if (intersect_id_start == intersect_id_end) return path;
    typedef double DVal; //short for Dijkstra Value which is the temporary value assigned to vertices when the algorithm is working
    typedef std::pair<DVal, IntersectionIndex> Node;


    std::vector<DVal> dijkstraValue;
    std::vector<std::pair<StreetSegmentIndex, IntersectionIndex>> parents;
    std::priority_queue<Node, std::deque<Node>, std::greater < Node>> heap;
    dijkstraValue.clear();
    dijkstraValue.resize(getNumberOfIntersections(), std::numeric_limits<double>::infinity());
    parents.clear();
    parents.resize(getNumberOfIntersections());
    std::unordered_set<unsigned> target_list;


    target_list.insert(intersect_id_end);


    heap.push(std::make_pair(0, intersect_id_start));
    dijkstraValue[intersect_id_start] = 0;
    IntersectionIndex currentNode;
    while (!heap.empty()) {
        currentNode = heap.top().second;
        heap.pop();


        std::unordered_set<unsigned>::iterator element = target_list.find(currentNode);
        if (element != target_list.end()) {
            break;
        }

        //get all adjacent nodes to current node
        std::deque <Node_Edge_Object> node_list = adjacency_list[currentNode];
        for (unsigned i = 0; i < node_list.size(); i++) {
            double cost = dijkstraValue[currentNode] + node_list[i].weight;

            //cost = cost + (find_distance_between_two_points(getIntersectionPosition(currentNode), getIntersectionPosition(intersect_id_end)));
            //insert stuff about turn penalties here
            StreetSegmentInfo prevStreetSeg = getStreetSegmentInfo(parents[currentNode].first);
            StreetSegmentInfo newStreetSeg = getStreetSegmentInfo(node_list[i].SegId);
            if (prevStreetSeg.streetID != newStreetSeg.streetID && currentNode != intersect_id_start) {
                cost = cost + turn_penalty;
            }



            if (dijkstraValue[node_list[i].id] > cost) {
                dijkstraValue[node_list[i].id] = cost;
                heap.push(std::make_pair(dijkstraValue[node_list[i].id], node_list[i].id));
                parents[node_list[i].id] = std::make_pair(node_list[i].SegId, currentNode);
            }
        }
    }
    if (currentNode != intersect_id_end) {
        return path;
    }
    if (dijkstraValue[currentNode] != std::numeric_limits<double>::infinity()) {
        IntersectionIndex index = currentNode;
        std::deque<Node_Edge_Object> node_list = adjacency_list[index];
        while (index != intersect_id_start) {
            path.push_back(parents[index].first);
            index = parents[index].second;
        }

    }
    std::reverse(path.begin(), path.end());
    auto time2 = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - start_time).count() << "\n";
    return path;
}

std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start,
        std::vector<unsigned> targets,
        const double turn_penalty) {
    std::vector<unsigned> path;
    typedef double DVal; //short for Dijkstra Value which is the temporary value assigned to vertices when the algorithm is working
    typedef std::pair<DVal, IntersectionIndex> Node;


    std::vector<DVal> dijkstraValue;
    std::vector<std::pair<StreetSegmentIndex, IntersectionIndex>> parents;
    std::priority_queue<Node, std::deque<Node>, std::greater < Node>> heap;
    dijkstraValue.clear();
    dijkstraValue.resize(getNumberOfIntersections(), std::numeric_limits<double>::infinity());
    parents.clear();
    parents.resize(getNumberOfIntersections());
    std::unordered_set<unsigned> target_list(targets.begin(), targets.end());


    heap.push(std::make_pair(0, intersect_id_start));
    dijkstraValue[intersect_id_start] = 0;
    IntersectionIndex currentNode;
    while (!heap.empty()) {


        currentNode = heap.top().second;
        heap.pop();


        std::unordered_set<unsigned>::iterator element = target_list.find(currentNode);
        if (element != target_list.end()) {
            break;
        }

        //get all adjacent nodes to current node
        std::deque <Node_Edge_Object> node_list = adjacency_list[currentNode];
        for (unsigned i = 0; i < node_list.size(); i++) {
            double cost = dijkstraValue[currentNode] + node_list[i].weight;

            //cost = cost + (find_distance_between_two_points(getIntersectionPosition(currentNode), getIntersectionPosition(intersect_id_end)));
            //insert stuff about turn penalties here
            StreetSegmentInfo prevStreetSeg = getStreetSegmentInfo(parents[currentNode].first);
            StreetSegmentInfo newStreetSeg = getStreetSegmentInfo(node_list[i].SegId);
            if (prevStreetSeg.streetID != newStreetSeg.streetID && currentNode != intersect_id_start) {
                cost = cost + turn_penalty;
            }



            if (dijkstraValue[node_list[i].id] > cost) {
                dijkstraValue[node_list[i].id] = cost;
                heap.push(std::make_pair(dijkstraValue[node_list[i].id], node_list[i].id));
                parents[node_list[i].id] = std::make_pair(node_list[i].SegId, currentNode);
            }
        }
    }
    if (target_list.find(currentNode) == target_list.end()) {

        return path;
    }
    if (dijkstraValue[currentNode] != std::numeric_limits<double>::infinity()) {
        IntersectionIndex index = currentNode;
        std::deque<Node_Edge_Object> node_list = adjacency_list[index];
        while (index != intersect_id_start) {
            path.push_back(parents[index].first);
            index = parents[index].second;
        }
    }
    std::reverse(path.begin(), path.end());

    return path;
}

// Returns the shortest travel time path (vector of street segments) from 
// the start intersection to a point of interest with the specified name.
// The path will begin at the specified intersection, and end on the 
// intersection that is closest (in Euclidean distance) to the point of 
// interest.
// If no such path exists, returns an empty (size == 0) vector.

std::vector<unsigned> find_path_to_point_of_interest(const unsigned intersect_id_start,
        const std::string point_of_interest_name,
        const double turn_penalty) {


    std::vector<POIIndex> pathfinding_targets_1 = POI_list[point_of_interest_name];
    std::vector<IntersectionIndex> pathfinding_targets;
    for (unsigned i = 0; i < pathfinding_targets_1.size(); i++) {
        unsigned id = find_closest_intersection(getPointOfInterestPosition(pathfinding_targets_1[i])); //index of POI
        pathfinding_targets.push_back(id);

    }

    std::vector<unsigned> path = find_path_between_intersections(intersect_id_start, pathfinding_targets, turn_penalty);

    return path;
}

double determinant(std::pair<LatLon, LatLon> begin, std::pair<LatLon, LatLon> end) {
    double x1 = lon_to_x(begin.first);
    double x2 = lon_to_x(begin.second);
    double x3 = lon_to_x(end.first);
    double x4 = lon_to_x(end.second);
    double y1 = lat_to_y(begin.first);
    double y2 = lat_to_y(begin.second);
    double y3 = lat_to_y(end.first);
    double y4 = lat_to_y(end.second);

    return ((x2 - x1)*(y4 - y3) - (y2 - y1)*(x4 - x3));
}

std::vector<std::string> all_turns(std::vector<unsigned> path) {
    std::vector<std::string> path_directions;
    if (path.empty())
        return path_directions;

    int direction_turn_count = 0;
    std::vector<double> straight_distances;
    double straight_distance = 0;
    for (unsigned i = 0; i < path.size() - 1; i++) {
        StreetSegmentInfo s1 = getStreetSegmentInfo(path[i]);
        straight_distance = straight_distance + find_street_segment_length(path[i]);
        StreetSegmentInfo s2 = getStreetSegmentInfo(path[i + 1]);
        if (s1.streetID != s2.streetID) {
            direction_turn_count++;
            straight_distances.push_back(straight_distance);
            straight_distance = 0;
        }
    }
    straight_distance = straight_distance + find_street_segment_length(path[path.size() - 1]);
    straight_distances.push_back(straight_distance);
    //Calculate the determinant between current path and next path
    //resulting vector will be in z-direction, if it = 0 then straight
    //-ve for right and +ve for left
    //A is the current segment
    //B is the next segment
    //A(x,y)xB(x,y) = z_(AxBy - BxAy)

    unsigned i;
    double z_val;
    //PUT CARDINAL DIRECTION HERE ASWELL
    /* StreetSegmentInfo s = getStreetSegmentInfo(path[0]);
     LatLon inter1 = getIntersectionPosition(s.from);
     LatLon inter2 = getIntersectionPosition(s.to);
     std::string cardinal_dir;
     if (s.curvePointCount != 0) {
         inter2 = getStreetSegmentCurvePoint(path[0], 0);
     }
     double angle = atan2(lat_to_y(inter2) - lat_to_y(inter1), lon_to_x(inter2) - lon_to_x(inter1)) / DEG_TO_RAD;
     std::cout << lat_to_y(inter2) - lat_to_y(inter1) << " " << lon_to_x(inter2) - lon_to_x(inter1) << "\n";
     std::cout << angle << "\n";
     if (angle > 135 && angle < -135){
         cardinal_dir = "West";
     }
     else if (angle > -135 && angle <-45){
         cardinal_dir = "South";
     }
     else if (angle > 45 && angle <135){
         cardinal_dir = "North";
     }
     else{
         cardinal_dir = "East";
     }*/

    if (getStreetName(getStreetSegmentInfo(path[0]).streetID) != "<unknown>") {
        std::string dist = boost::lexical_cast<std::string>(round(straight_distances[0]));
        path_directions.push_back("Head onto " + getStreetName(getStreetSegmentInfo(path[0]).streetID) + " for " + dist + " m");
    } else {
        path_directions.push_back("Head towards " + getStreetName(getStreetSegmentInfo(path[1]).streetID));
    }


    int counter = 0;
    for (i = 0; i < path.size() - 1; ++i) {
        //path[i] // gives current segment
        //path[i+1] //gives next segment
        //take the x and y coordinates and calculate its determinant

        unsigned starting_intersection;
        unsigned common_intersection;
        unsigned ending_intersection;

        //Gets the current and next segments 
        StreetSegmentInfo seg1 = getStreetSegmentInfo(path[i]);
        StreetSegmentInfo seg2 = getStreetSegmentInfo(path[i + 1]);
        StreetSegmentInfo previousSeg;
        if (i > 0)
            previousSeg = getStreetSegmentInfo(path[i - 1]);
        //Checking for the common node
        if (seg1.to == seg2.from) {

            starting_intersection = seg1.from;
            common_intersection = seg1.to;
            ending_intersection = seg2.to;

        } else if (seg1.from == seg2.to) {

            starting_intersection = seg1.to;
            common_intersection = seg1.from;
            ending_intersection = seg2.from;

        } else if (seg1.to == seg2.to) {

            starting_intersection = seg1.from;
            common_intersection = seg1.to;
            ending_intersection = seg2.from;

        } else {

            starting_intersection = seg1.to;
            common_intersection = seg1.from;
            ending_intersection = seg2.to;
        }

        std::pair<LatLon, LatLon> begin_street;
        begin_street.first = getIntersectionPosition(starting_intersection);
        begin_street.second = getIntersectionPosition(common_intersection);
        std::pair<LatLon, LatLon> end_street;
        end_street.first = getIntersectionPosition(common_intersection);
        end_street.second = getIntersectionPosition(ending_intersection);

        z_val = determinant(begin_street, end_street);


        if (seg1.streetID != seg2.streetID) {
            if (z_val > 0) {
                if (seg1.streetID != seg2.streetID) {
                    if (getStreetName(seg2.streetID) != "<unknown>") {
                        path_directions.push_back("Turn left onto " + getStreetName(seg2.streetID));

                    } else {
                        path_directions.push_back("Turn left ");
                    }
                    //path_directions.push_back("left");
                } else {
                    ;
                }
            } else if (z_val < 0) {
                if (seg1.streetID != seg2.streetID) {
                    if (getStreetName(seg2.streetID) != "<unknown>") {
                        path_directions.push_back("Turn right onto " + getStreetName(seg2.streetID));

                    } else {
                        path_directions.push_back("Turn right");
                    }
                    //path_directions.push_back("right");
                } else {
                    ;
                }
            }
            counter++;
        } else {
            if (i > 0) {
                if (seg1.streetID != previousSeg.streetID) {
                    std::string dist = boost::lexical_cast<std::string>(round(straight_distances[counter]));
                    if (getStreetName(seg2.streetID) != "<unknown>") {

                        path_directions.push_back("Continue on " + getStreetName(seg1.streetID) + " for " + dist + " m");

                    } else {

                        path_directions.push_back("Continue heading straight on current road for " + dist + " m");
                    }
                    straight_distances[counter] = straight_distances[counter] - find_street_segment_length(path[i]);
                } else {
                    ;
                } // skip
            }
        }
    }
    path_directions.push_back("You have arrived at your destination");
    return path_directions;
}
