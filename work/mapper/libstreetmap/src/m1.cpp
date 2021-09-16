/* 
 * Copyright 2018 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "OSMEntity.h"
#include "OSMID.h"
#include "OSMDatabaseAPI.h"
#include "OSMNode.h"
#include "OSMWay.h"
#include "OSMRelation.h"
#include "m1.h"
#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include "global_functions.h"
#include <math.h>
#include <string>
#include <vector>
#include <list>
#include <algorithm>
#include <cstring>
#include <iterator>
#include <unordered_map>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
//#include <boost/sort/spreadsort/spreadsort.hpp>
#include <set>
#include <chrono>



#define RTREE_BOX_RANGE 0.00000000000000000000001
#define RTREE_POINT_RANGE  0.000000000000001
#define FACTOR_TO_MULTIPLY 0.27777777777777777778
#define PTHREAD_ENABLE 1

//Making Name spaces for boost class
//Also type defining specific names for ease of access

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;



//===========================GLOBAL DATA STRUCTURES=============================
//--Describe each data structures-- And mention why its global -> needed by other functions and
//                                  wouldn't pass performance if made in there.

/*1: A map of where keys are street names which stores a vector of all street_ids
 * associated with a certain name
 */

/*2: A vector of vectors which stores intersections id's and then for each 
 * intersection id it stores a vector of street segment id's associated with that 
 * intersection id
 */

/*3: A vector of travel times for each street segment, stored as a global
 * doing the calculations in the function itself would take too long and fail
 * performance tests 
 */

/*4: A unordered map which takes a street name as the key and return a vector 
 * of intersection id's associated with that name 
 */

/*5: A un-ordered map which takes in a street id as the key and returns all a vector
 *  of all street segments associated with that street_id    
 */

/*6: A r-tree that stores intersection coordinates (LatLon) points, each box only 
 * contains one point
 */

/*7: A r-tree that stores points of intersect, each box only contains one point
 * intersect 
 */

/*1*/std::unordered_map<std::string, std::vector<unsigned>> streetID_from_streetName;
/*2*/std::vector<std::vector<unsigned>> Intersections_Street_Segments;
/*3*/std::vector<double> Travel_Times;
/*4*/std::unordered_map<std::string, std::vector<unsigned>> Street_Name_Intersection;
/*5*/std::unordered_map<unsigned, std::vector<StreetSegmentIndex>> StreetID_Segments;
/*6*/bgi::rtree<value, bgi::linear < 16 >> Intersection_RTree;
/*7*/bgi::rtree<value, bgi::linear < 16 >> POI_RTree;
bgi::rtree<value, bgi::linear < 16 >> segments_RTree;
bgi::rtree<value, bgi::linear < 16 >> f_rtree;
std::vector<std::deque < Node_Edge_Object>> adjacency_list;
std::unordered_map<std::string, std::vector<unsigned>> POI_list;
std::unordered_map<int, char> keypress_LUT;
void load_LUT();
void* intersection_loop(void *dummy);
void* poi_loop(void *dummy);
void* features_loop(void *dummy);
void* segments_loop(void *dummy);
void* load_layer1(void *path);
double min_lat;
double max_lat;
double min_lon;
double max_lon;
double average_lat_again;

void* load_layer1(void *path) {
    std::string *path_string = static_cast<std::string*> (path);
    loadOSMDatabaseBIN(*path_string);
    pthread_exit(NULL);
}

/*  In this function, data gets written into all the appropriate structures
 *  @param map_name: This variable saves the path for the where the file is contained
 *  @return true or false, enabling us to if the map loaded successfully or not */
bool load_map(std::string map_name/*map_path*/) {
    
    bool load_successful = false; //Indicates whether the map has loaded 
    //successfully
    int pos = map_name.find(".streets");
    std::string osm_map_name = map_name;
    osm_map_name.replace(pos, 8, ".osm");

    pthread_t load_thread;
    pthread_attr_t load_attr;
    void *load_status;
    pthread_attr_init(&load_attr);
    pthread_attr_setdetachstate(&load_attr, PTHREAD_CREATE_JOINABLE);
    void* arg = (void *) &osm_map_name;
    pthread_create(&load_thread, NULL, load_layer1, arg);


    //loadOSMDatabaseBIN(osm_map_name);


    if (!loadStreetsDatabaseBIN(map_name)) {
        return false;
    }

    load_LUT();

    max_lat = getIntersectionPosition(0).lat();
    min_lat = max_lat;
    max_lon = getIntersectionPosition(0).lon();
    min_lon = max_lon;
    average_lat_again = 0;
    int count1 = 0;
    for (unsigned i = 0; i < getNumberOfFeatures(); i++) {
        for (unsigned j = 0; j < getFeaturePointCount(i); j++) {
            average_lat_again = average_lat_again + getFeaturePoint(i, j).lat();
            count1++;
        }
    }
    for (unsigned p = 0; p < getNumberOfIntersections(); ++p) {
        average_lat_again += getIntersectionPosition(p).lat();
    }
    average_lat_again = average_lat_again * DEG_TO_RAD / (getNumberOfIntersections() + count1);

    Intersections_Street_Segments.resize(getNumberOfIntersections());
    //for loop to iterate through all intersections. In this for loop we write
    //to both Intersections_Street_Segments and Intersection_RTree


    pthread_t threads[4];
    pthread_attr_t attr;
    void *status;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    pthread_create(&threads[0], NULL, intersection_loop, (void *) 0);
    //intersection_loop(NULL);

    pthread_create(&threads[1], NULL, poi_loop, (void *) 1);
    //poi_loop(NULL);

    //fills rtree for features
    pthread_create(&threads[2], NULL, features_loop, (void *) 2);
    //features_loop(NULL);


    adjacency_list.resize(getNumberOfIntersections());
    pthread_create(&threads[3], NULL, segments_loop, (void *) 3);
    //segments_loop(NULL);

    pthread_attr_destroy(&attr);
    pthread_join(threads[0], &status);
    pthread_join(threads[1], &status);
    pthread_join(threads[2], &status);
    pthread_join(threads[3], &status);



    for (unsigned streetId = 0; streetId < getNumberOfStreets(); ++streetId) {
        std::string street_name = getStreetName(streetId);
        std::vector<unsigned> intersections = find_all_street_intersections(streetId);
        //Puts all the intersections associated with the name under same key in data structure.
        Street_Name_Intersection[street_name].insert(Street_Name_Intersection[street_name].end(),
                intersections.begin(), intersections.end());
    }
    last_map_name(map_name);
    load_successful = true; //Make sure this is updated to reflect whether
    //loading the map succeeded or failed.
    pthread_attr_destroy(&load_attr);
    
    pthread_join(load_thread, &load_status);
 
    //std::cout << time_end - time_start << " layer1 wait time\n";
    return load_successful;
}

void* intersection_loop(void *dummy) {
    for (unsigned intersection = 0; intersection < getNumberOfIntersections(); ++intersection) {

        max_lat = std::max(max_lat, getIntersectionPosition(intersection).lat());
        min_lat = std::min(min_lat, getIntersectionPosition(intersection).lat());

        max_lon = std::max(max_lon, getIntersectionPosition(intersection).lon());
        min_lon = std::min(min_lon, getIntersectionPosition(intersection).lon());

        //this for loop iterates through each segment in the intersection
        for (unsigned i = 0; i < getIntersectionStreetSegmentCount(intersection); ++i) {
            StreetSegmentIndex const ss_id = getIntersectionStreetSegment(intersection, i);
            //Puts all segments associated with intersection into data structure
            Intersections_Street_Segments[intersection].push_back(ss_id);
        }
        //Filling up the Intersection_Rtree:
        LatLon coordinate = getIntersectionPosition(intersection);
        //The box is created by adding a small offset to the coordinate. Point 1
        //is the bottom left corner and Point 2 is the top right corner. The offset
        //needs to be as small as possible to avoid overlaps with boxes around
        //other intersections. This increases query speed and reduces error in
        //the find algorithm.
        box b(point(coordinate.lon() * cos(average_lat_again) * DEG_TO_RAD - RTREE_BOX_RANGE, coordinate.lat() * DEG_TO_RAD - RTREE_BOX_RANGE),
                point(coordinate.lon() * cos(average_lat_again) * DEG_TO_RAD + RTREE_BOX_RANGE, coordinate.lat() * DEG_TO_RAD + RTREE_BOX_RANGE));

        Intersection_RTree.insert(std::make_pair(b, intersection));

    }


    pthread_exit(NULL);
}

void* poi_loop(void *dummy) {
    for (unsigned indexPOI = 0; indexPOI < getNumberOfPointsOfInterest(); ++indexPOI) {
        LatLon coordinate = getPointOfInterestPosition(indexPOI);
        //Same idea as the Intersection_RTree but for POI.
        box b(point(coordinate.lon() * cos(average_lat_again) * DEG_TO_RAD - RTREE_BOX_RANGE, coordinate.lat() * DEG_TO_RAD - RTREE_BOX_RANGE),
                point(coordinate.lon() * cos(average_lat_again) * DEG_TO_RAD + RTREE_BOX_RANGE, coordinate.lat() * DEG_TO_RAD + RTREE_BOX_RANGE));
        POI_RTree.insert(std::make_pair(b, indexPOI));

        POI_list[getPointOfInterestName(indexPOI)].push_back(indexPOI);
    }


    pthread_exit(NULL);
}

void* features_loop(void *dummy) {
    for (unsigned i = 0; i < getNumberOfFeatures(); i++) {
        //coordinates to for box around feature
        double lx = getFeaturePoint(i, 0).lon();
        double rx = getFeaturePoint(i, 0).lon();
        double ty = getFeaturePoint(i, 0).lat();
        double by = getFeaturePoint(i, 0).lat();
        for (unsigned j = 0; j < getFeaturePointCount(i); j++) {
            if (std::min(getFeaturePoint(i, j).lon(), lx) != lx) {
                lx = getFeaturePoint(i, j).lon();
            }
            if (std::max(getFeaturePoint(i, j).lon(), rx) != rx) {
                rx = getFeaturePoint(i, j).lon();
            }
            if (std::min(getFeaturePoint(i, j).lat(), by) != by) {
                by = getFeaturePoint(i, j).lat();
            }
            if (std::max(getFeaturePoint(i, j).lat(), ty) != ty) {
                ty = getFeaturePoint(i, j).lat();
            }
        }
        box b(point(lx * cos(average_lat_again) * DEG_TO_RAD, by * DEG_TO_RAD),
                point(rx * cos(average_lat_again) * DEG_TO_RAD, ty * DEG_TO_RAD));
        /*max_lat = std::max(max_lat, ty);
        min_lat = std::min(min_lat, by);

        max_lon = std::max(max_lon, rx);
        min_lon = std::min(min_lon, lx);*/
        f_rtree.insert(std::make_pair(b, i));
    }


    pthread_exit(NULL);
}

void* segments_loop(void *dummy) {

    for (StreetSegmentIndex i = 0; i < getNumberOfStreetSegments(); ++i) {
        /* Calculate each segments travel time and put it in data structure
         * Adds Segment Travel times for each segment into a vector "Travel_Times"
         * Stores street segments associated with each street id in unordered map
         * "StreetID_Segments".    */
        StreetSegmentInfo tempInfo = getStreetSegmentInfo(i);

        StreetID_Segments[tempInfo.streetID].push_back(i);


        Travel_Times.push_back(find_street_segment_length(i) *
                (1.0 / ((tempInfo.speedLimit) * FACTOR_TO_MULTIPLY)));
        //Check to see if already inserted before.

        if (std::count(streetID_from_streetName[getStreetName(tempInfo.streetID)].begin(),
                streetID_from_streetName[getStreetName(tempInfo.streetID)].end(),
                tempInfo.streetID) == 0) {
            streetID_from_streetName[getStreetName(tempInfo.streetID)].push_back(tempInfo.streetID);
        }

        LatLon p1 = getIntersectionPosition(tempInfo.from);
        LatLon p2 = getIntersectionPosition(tempInfo.to);
        double x1 = p1.lon() * cos(average_lat_again) * DEG_TO_RAD;
        double y1 = p1.lat() * DEG_TO_RAD;
        double x2 = p2.lon() * cos(average_lat_again) * DEG_TO_RAD;
        double y2 = p2.lat() * DEG_TO_RAD;
        if (x1 > x2 && y1 > y2) {
            box b(point(x2, y2), point(x1, y1));
            segments_RTree.insert(std::make_pair(b, i));
        } else if (x1 < x2 && y1 > y2) {
            box b(point(x1, y2), point(x2, y1));
            segments_RTree.insert(std::make_pair(b, i));
        } else if (x1 > x2 && y1 < y2) {
            box b(point(x2, y1), point(x1, y2));
            segments_RTree.insert(std::make_pair(b, i));
        } else if (x1 < x2 && y1 < y2) {
            box b(point(x1, y1), point(x2, y2));
            segments_RTree.insert(std::make_pair(b, i));
        } else if (x1 == x2) {
            box b(point(x2 - RTREE_POINT_RANGE, std::min(y1, y2)), point(x1 + RTREE_POINT_RANGE, std::max(y1, y2)));
            segments_RTree.insert(std::make_pair(b, i));
        } else if (y1 == y2) {
            box b(point(std::min(x1, x2), y2 - RTREE_POINT_RANGE), point(std::max(x1, x2), y1 + RTREE_POINT_RANGE));
            segments_RTree.insert(std::make_pair(b, i));
        }


        //adjacency list
        Node_Edge_Object tempObj;
        tempObj.id = tempInfo.to;
        std::vector<unsigned> SegmentIndex;
        SegmentIndex.push_back(i);
        tempObj.weight = compute_path_travel_time(SegmentIndex, 0);
        tempObj.SegId = i;
        adjacency_list[tempInfo.from].push_back(tempObj);
        if (tempInfo.oneWay != true) {
            tempObj.id = tempInfo.from;
            adjacency_list[tempInfo.to].push_back(tempObj);
        }
    }


    pthread_exit(NULL);
}

/** This function is for clearing all the data structures and closing the map itself.
 *  @param void because no inputs needed
 *  @return nothing because not needed */
void close_map() {
    //Clean-up the map related data structures here.
    closeOSMDatabase();
    closeStreetDatabase();
    streetID_from_streetName.clear();
    Intersections_Street_Segments.clear();
    Travel_Times.clear();
    StreetID_Segments.clear();
    Street_Name_Intersection.clear();
    Intersection_RTree.clear();
    POI_RTree.clear();
    f_rtree.clear();
    segments_RTree.clear();
    adjacency_list.clear();

}

/** Gets the streets ids associated with the street name given through the data structure.
 *  @param street_name: Name of the street
 *  @return  a vector of all the ids associated with the name */
std::vector<unsigned> find_street_ids_from_name(std::string street_name) {
    return streetID_from_streetName[street_name];
}

void load_LUT() {
    //enter is 36, shift is 50, backspace is 22, ampersand is 16, space is 65
    keypress_LUT[24] = 'q';
    keypress_LUT[25] = 'w';
    keypress_LUT[26] = 'e';
    keypress_LUT[27] = 'r';
    keypress_LUT[28] = 't';
    keypress_LUT[29] = 'y';
    keypress_LUT[30] = 'u';
    keypress_LUT[31] = 'i';
    keypress_LUT[32] = 'o';
    keypress_LUT[33] = 'p';
    keypress_LUT[38] = 'a';
    keypress_LUT[39] = 's';
    keypress_LUT[40] = 'd';
    keypress_LUT[41] = 'f';
    keypress_LUT[42] = 'g';
    keypress_LUT[43] = 'h';
    keypress_LUT[44] = 'j';
    keypress_LUT[45] = 'k';
    keypress_LUT[46] = 'l';
    keypress_LUT[52] = 'z';
    keypress_LUT[53] = 'x';
    keypress_LUT[54] = 'c';
    keypress_LUT[55] = 'v';
    keypress_LUT[56] = 'b';
    keypress_LUT[57] = 'n';
    keypress_LUT[58] = 'm';
}

/** Gets street segments associated with the intersection's id from a data structure.
 *  @param intersection_id is the intersection id
 *  @return a vector of street segment ids associated with the intersection */
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id) {
    return Intersections_Street_Segments[intersection_id];
}

/** Gets street names associated with the intersection's id by looping through all data given.
 *  @param intersection_id is the intersection id
 *  @return a vector of street segment names associated with that intersection ID */
std::vector<std::string> find_intersection_street_names(unsigned intersection_id) {
    std::vector<std::string> streetNames;

    unsigned numOfStreetSeg = getIntersectionStreetSegmentCount(intersection_id);

    //Iterates through street segments, then gets the street segment info and 
    //stores the name into the vector and then returns the vector.
    for (unsigned i = 0; i < numOfStreetSeg; ++i) {
        StreetSegmentIndex tempIndex = getIntersectionStreetSegment(intersection_id, i);
        StreetSegmentInfo temp = getStreetSegmentInfo(tempIndex);
        streetNames.push_back(getStreetName(temp.streetID));
    }

    return streetNames;
}

/** Checks to see if you can get from intersection 1 to intersection 2 using just 1 street
 *  segment 
 *  @param intersection_id1 is an intersections id that will be used to check connection
 *  @param intersection_id2 is an intersections id that will be used to check connection
 *  @return true or false depending if connected or not */
bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2) {

    unsigned intersectionOne = getIntersectionStreetSegmentCount(intersection_id1);
    unsigned intersectionTwo = getIntersectionStreetSegmentCount(intersection_id2);

    //Corner case: if the intersection is the same 
    if (intersection_id1 == intersection_id2) {
        return true;
    }

    /* Idea:
     *  Goes through all street segments in both intersections 
     *  until it finds a street segment both streets share 
     *  then checks to see if you can safely go from 
     *  intersection 1(one-way check) to intersection two
     *  if found it returns true
     *  else returns false */
    for (unsigned intersection_index1 = 0; intersection_index1 < intersectionOne; ++intersection_index1) {

        for (unsigned intersection_index2 = 0; intersection_index2 < intersectionTwo; ++intersection_index2) {

            if (getIntersectionStreetSegment(intersection_id1, intersection_index1) ==
                    getIntersectionStreetSegment(intersection_id2, intersection_index2)) {

                StreetSegmentIndex temp = getIntersectionStreetSegment(intersection_id1, intersection_index1);
                StreetSegmentInfo tempInfo = getStreetSegmentInfo(temp);
                if (tempInfo.oneWay != true) {
                    return true;
                } else if ((tempInfo.from == intersection_id1 && tempInfo.to == intersection_id2) ||
                        (tempInfo.to == intersection_id1 && tempInfo.from == intersection_id2)) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    }
    return false;
}

/** Returns all intersections you can reach from traveling down
 *  just one street segment  
 *  @param intersection_id1:the "from" intersection 
 *  @return returns a vector of intersection id's  that are adjacent to intersection_id*/
std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id) {

    std::vector<unsigned> adjacentIntersections;

    unsigned tempStreetSeg = getIntersectionStreetSegmentCount(intersection_id);

    /* Idea:
     *  Goes through all street segments connected to the intersection 
     *  checks to see if intersection is "to" or "from", if it is "to"
     *  checks to see if road is one way or not.
     *  Finally checks to see if connected intersection is already in the vector
     *  if all tests pass then adds intersection to vector of intersections ids. */
    for (unsigned i = 0; i < tempStreetSeg; ++i) {

        StreetSegmentIndex tempIndex = getIntersectionStreetSegment(intersection_id, i);
        StreetSegmentInfo tempInfo = getStreetSegmentInfo(tempIndex);
        std::vector<unsigned>::iterator itTo = std::find(adjacentIntersections.begin(), adjacentIntersections.end(), tempInfo.to);
        std::vector<unsigned>::iterator itFrom = std::find(adjacentIntersections.begin(), adjacentIntersections.end(), tempInfo.from);

        if (tempInfo.from == intersection_id && itTo == adjacentIntersections.end()) {
            adjacentIntersections.push_back(tempInfo.to);
        } else if (tempInfo.to == intersection_id && tempInfo.oneWay == false && itFrom == adjacentIntersections.end()) {
            adjacentIntersections.push_back(tempInfo.from);
        }
    }
    return adjacentIntersections;
}

/** This function gives all the street segments in a given street, goes into
 *  unordered map and gets vector from there.
 *  @param street_id, the street whose street segments will be returned  
 *  @return vector of street segment id's which are found in street_id*/
std::vector<unsigned> find_street_street_segments(unsigned street_id) {
    return StreetID_Segments[street_id];
}

/** Finds all the intersections associated with a streets id by going through its street segment.
 *  @param street_id is the id of the street
 *  @return a vector of all the intersection ids found  */
std::vector<unsigned> find_all_street_intersections(unsigned street_id) {

    std::vector<unsigned> street_intersections_id;

    /** Calls "find_street_street_segment" function and iterates
     *  through that vector, checks all intersections to see if the "to" and
     *  "from" intersections have been added to the return vector, if not then adds it.
     */

    std::vector<unsigned> segments = find_street_street_segments(street_id);

    for (std::vector<unsigned>::iterator segments_it = segments.begin();
            segments_it != segments.end(); ++segments_it) {
        StreetSegmentInfo seg = getStreetSegmentInfo(*segments_it);

        street_intersections_id.push_back(seg.from);
        street_intersections_id.push_back(seg.to);

    }
    std::sort(street_intersections_id.begin(), street_intersections_id.end());
    street_intersections_id.erase(std::unique(street_intersections_id.begin(), street_intersections_id.end()), street_intersections_id.end());
    return street_intersections_id;
}

/** Finds all the intersection ids from a map using the names of the streets that make the intersection.
 *  @param street_name1 a string of the one of the streets making the intersection
 *  @param street_name2 a string of the other street making the intersection
 *  @return a vector of all the intersections ids found*/
std::vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1,
        std::string street_name2) {

    std::unordered_map<std::string, std::vector<unsigned>>::iterator name_1 =
            Street_Name_Intersection.find(street_name1);
    std::unordered_map<std::string, std::vector<unsigned>>::iterator name_2 =
            Street_Name_Intersection.find(street_name2);

    std::vector<unsigned> intersection_ids;
    if (name_1 == Street_Name_Intersection.end()) {
        //error message of street_name1 not a real street
        return intersection_ids;
    }
    if (name_2 == Street_Name_Intersection.end()) {
        //error message of street_name2 not a real street
        return intersection_ids;
    }
    std::vector<unsigned> street1_intersection_ids = (*name_1).second;
    std::vector<unsigned> street2_intersection_ids = (*name_2).second;
    for (std::vector<unsigned>::iterator street1_intersection_ids_it = street1_intersection_ids.begin();
            street1_intersection_ids_it != street1_intersection_ids.end(); ++street1_intersection_ids_it) {

        if (std::find(street2_intersection_ids.begin(), street2_intersection_ids.end(),
                *street1_intersection_ids_it) != street2_intersection_ids.end()) {

            intersection_ids.push_back(*street1_intersection_ids_it);
        }
    }
    return intersection_ids;
}

/** Finds distance between two points given on the map in meters using equations 
 *  given to us.
 *  @param LatLon point1: 1st point 
 *  @param LatLon point2: 2nd point 
 *  @return distance (in meters) between point 1 and point 2
 */
double find_distance_between_two_points(LatLon point1, LatLon point2) {

    double x1, x2, y1, y2;
    double lat_avg = ((point1.lat() + point2.lat()) *0.5) * DEG_TO_RAD;
    x1 = (point1.lon() * DEG_TO_RAD) * cos(lat_avg);
    y1 = point1.lat() * DEG_TO_RAD;
    x2 = (point2.lon() * DEG_TO_RAD) * cos(lat_avg);
    y2 = point2.lat() * DEG_TO_RAD;
    return EARTH_RADIUS_IN_METERS * sqrt(((y2 - y1)*(y2 - y1)+(x2 - x1)*(x2 - x1)));
}

/** Finds length of a street segment by using "find_distance_between_two_points". 
 *  @param street_segment_id, the street segment  
 *  @return distance of a street segment in meters*/
double find_street_segment_length(unsigned street_segment_id) {
    StreetSegmentInfo s = getStreetSegmentInfo(street_segment_id);
    int curve_points = s.curvePointCount;
    if (curve_points == 0) {
        return find_distance_between_two_points(getIntersectionPosition(s.from),
                getIntersectionPosition(s.to));
    }
    /* Iterates through curve points, find distance between them and adds that to
     * the already existing sum. */
    double sum = find_distance_between_two_points(getIntersectionPosition(s.from),
            getStreetSegmentCurvePoint(street_segment_id, 0));
    int i;
    for (i = 0; i < curve_points - 1; ++i) {
        sum = sum + find_distance_between_two_points(getStreetSegmentCurvePoint(street_segment_id, i),
                getStreetSegmentCurvePoint(street_segment_id, i + 1));
    }
    sum = sum + find_distance_between_two_points(getStreetSegmentCurvePoint(street_segment_id, i),
            getIntersectionPosition(s.to));
    return sum;
}

/** Finds the length of a street in meters and returns that, goes through each 
 *  street segment and calls "find_street_segment_length" and adds it to the sum
 *  @param street_id, the street id 
 *  @return the length of the street in meters
 */
double find_street_length(unsigned street_id) {
    std::vector<unsigned> segment = find_street_street_segments(street_id);
    double sum = 0;
    for (unsigned i = 0; i < segment.size(); ++i) {
        sum = sum + find_street_segment_length(segment[i]);
    }
    return sum;
}

/** Finds the time taken to travel down a street segment bases on street segment 
 *  length and on the speed limit. Calls the vector "Travel_Times". 
 *  @param street_segment_id, the street segment id 
 *  @return time taken to travel down that street segment in seconds*/
double find_street_segment_travel_time(unsigned street_segment_id) {
    return Travel_Times[street_segment_id];
}

/** Uses built in nearest point query algorithm to find the closest POI in the tree
 *  to given point and returns the id of the element returned
 *  @param my_position being the position on earth given
 *  @return the Id of the closest POI*/
unsigned find_closest_point_of_interest(LatLon my_position) {
    std::vector<value> result;
    //Query function returns the closest point to location given
    POI_RTree.query(bgi::nearest(point(my_position.lon() * cos(average_lat_again) * DEG_TO_RAD, my_position.lat() * DEG_TO_RAD), 1), std::back_inserter(result));
    return result[0].second;
}

/** Same implementation as the find closest point of interest function for similar reason.
 *  @param my_position being the position on earth given
 *  @return the Id of the closest intersection */
unsigned find_closest_intersection(LatLon my_position) {
    std::vector<value> result;
    Intersection_RTree.query(bgi::nearest(point(my_position.lon() * cos(average_lat_again) * DEG_TO_RAD, my_position.lat() * DEG_TO_RAD), 5), std::back_inserter(result));
    unsigned index = 0;
    double lowest_dist = find_distance_between_two_points(my_position, getIntersectionPosition(result[0].second));
    for (unsigned i = 0; i < result.size(); i++) {
        double dist = find_distance_between_two_points(my_position, getIntersectionPosition(result[i].second));
        if (lowest_dist > dist) {
            index = i;
            lowest_dist = dist;
        }

    }

    return result[index].second;
}
