/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m3.h"
#include "m2.h"
#include "m1.h"
#include "LatLon.h"
#include <math.h>
#include "StreetsDatabaseAPI.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <OSMEntity.h>
#include <OSMID.h>
#include <OSMDatabaseAPI.h>
#include "OSMNode.h"
#include "OSMWay.h"
#include <unordered_map>
#include "OSMRelation.h"
#include <queue>
#include <limits>
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
#include <chrono>
#include <ctime>

/******************DEFINING CONSTANTS*************/
//Speed limit
#define HIGHWAY_SPEED_LIMIT 100
#define BIG_STREET_UPPER_LIMIT 80
#define BIG_STREET_LOWER_LIMIT 50
//Search constants
#define NO_SEARCH 0
#define INTERSECTION_SEARCH 1
#define POI_SEARCH 2
#define STREETS_SEARCH 3
#define CHOSEN_INTERSECTION_SEARCH 4
#define CHOSEN_POI_TO_INTERSECTION_SEARCH 5
#define CHOSEN_POI_SEARCH 6
//R_trees box and point
#define SMALL_BOX_RANGE 0.0000007
#define RADIUS_OF_CIRCLE 0.000005
//Font Sizes
#define FONT_SIZE_10 10
#define FONT_SIZE_12 12
#define FONT_SIZE_16 16
#define FONT_SIZE_18 18
//Turn penalty constant
#define TURN_PENALTY 15
//DID NOT DEFINE MAGIC NUMBERS FOR LOD's BECAUSE FELT IT WOULD OVER CLUTTER THE
//DEFINES AND WOULD HAVE MULTIPLES OF DEFINES BEING FACTORS OF 10 OFF. DID NOT
//WANT TO HAVE THE DIVISIONS IN THE FUNCTION AND THE MULTPLES WOULD REMOVE THE PURPOSE
//OF THE DEFINE FUNCTION.
/******************END OF DEFINING CONSTANTS*************/

/******************Defining Namespaces*******************/
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;
/*******************************************************/

/*************Global Variables***************/
struct Area_Points;
/********************************************/


/*************Function Prototypes***************/
void initalize_map();
void draw_screen();
void draw_street_segment(LatLon p1, LatLon p2);
void find_function(void (*drawscreen_ptr) ());
void change_map(void (*draw_map) ());
void search_help(void (*draw_map) ());
void closest_POI_search(void (*draw_screen) ());
void act_on_button_press(float x, float y, t_event_buttonPressed event);
void determine_screen();
void act_on_mousemove(float x, float y);
void display_name_and_coord(LatLon position, std::string name);
void draw_POI(double area, int i, std::vector<value> POI_to_draw);
void focus_level(double x, double y, double zoom_level);
void find_closest_street_segment(double x_coord, double y_coord);
void reset_highlights();
void draw_street_segment_highlight(std::vector<unsigned> to_draw, double &average_x,
        double &average_y, int &num);
void draw_street_highlight(std::vector<unsigned> to_draw, double &average_x,
        double &average_y, int &num);
std::vector<std::pair<FeatureIndex, Area_Points>> sort_feature_by_area();
bool sort_by_area(std::pair<FeatureIndex, Area_Points> temp1, std::pair<FeatureIndex, Area_Points> temp2);
LatLon to_spherical(float x, float y);
std::string street_classifier(StreetSegmentInfo s);
std::string capitalizing(std::string text);
std::string fix_string(std::string to_fix);
bool draw_segment(std::string street_type);
void sortOSMWaysbyOSMID();
/********************************************/


/*************Global Variables***************/
std::string last_map; //Map to return to if failed change
std::string object_name_1, object_name_2; //names of streets/intersections/POIs
std::string closest_POI_name; //The name of the POI searching for

//Area vector for all features
std::vector<std::pair<FeatureIndex, Area_Points>> featuresArea;
std::vector<std::string> directions; // stores all the directions for a path
//Found segments
std::vector<value> found_street_segments;

//Vectors to store saved searches, from intersections to points of interest
std::vector<unsigned> saved_intersections;
std::vector<unsigned> saved_POI;
std::vector<unsigned> saved_POI_to_intersection;
std::vector<unsigned> saved_intersection_to_POI;
std::vector<unsigned> found_search;
std::vector<unsigned> found_intersection_path;
std::vector<unsigned> found_POI_to_intersection_path;
std::vector<unsigned> found_intersection_to_POI_path;
std::vector<unsigned> found_POIs_path;
std::vector<unsigned> found_closest_POI_to_intersection_path;

double average_lat; //Sum intersection's latitudes divided by # of intersections
//Used to communicate between draw screen and searches
double intersection_x;
double intersection_y;

//Communicating between functions to know if it needs to be drawn or not
bool change_map_select = false;
bool opened_found_path = false;
bool do_closest_POI_search = false;
bool has_opened = false;
bool keep = false;
bool draw_closest_POI_to_intersection_path = false;
bool selected_intersection = false;
bool selected_POI = false;
bool draw_POI_path = false;
bool draw_intersection_path = false;
bool draw_POI_to_intersection_path = false;
bool draw_intersection_to_POI_path = false;
int search_method = 0; // 1 = intersection, 2 = POI, 3 = street name
std::unordered_map<OSMID, std::pair<const OSMWay*, unsigned>> OSMWayStreets;

// 4 = search chosen intersections
// 5 = search chosen POI to intersection
// 6 = search chosen POIs
int path_at;
int max_segments;

//Loading the png for drawing the pin for selecting intersections
Surface intersection_pin = load_png_from_file("libstreetmap/resources/pin_scaled");

//Loading the png for drawing the pin for selecting points of interest
Surface POI_pin = load_png_from_file("libstreetmap/resources/pin_scaled_b.png");

//Loading the close menu arrow
Surface close_menu_arrow = load_png_from_file("libstreetmap/resources/close_menu");
//Loading the open menu arrow
Surface open_menu_arrow = load_png_from_file("libstreetmap/resources/open_menu_scaled");

/********************************************/

struct Area_Points {
    double Featurearea;
    std::vector<t_point> featurePoints;
};

/* Sorts the OSMIDs and sets it up in an unordered_map
 * @param void no parameters needed
 * @return void nothing to return
 */
void sortOSMWaysbyOSMID() {

    std::pair<const OSMWay*, unsigned> temp;
    for (unsigned i = 0; i < getNumberOfWays(); ++i) {
        temp.first = getWayByIndex(i);
        for (unsigned j = 0; j < getTagCount(getWayByIndex(i)); ++j) {
            std::string key;
            key = getTagPair(getWayByIndex(i), j).first;
            temp.second = j;
            if (key == "highway") {
                OSMWayStreets[getWayByIndex(i)->id()].first = temp.first;
                OSMWayStreets[getWayByIndex(i)->id()].second = temp.second;
            }
        }
    }

}

/*  In this function, we iterate through all the features and find their area
 * then we store that into a pair of featureID and area
 *  @param void
 *  @return returns a vector of a pair of featureID's and their respective area*/
std::vector<std::pair<FeatureIndex, Area_Points>> sort_feature_by_area() {
    std::vector<std::pair<FeatureIndex, Area_Points>> featureArea;

    for (unsigned i = 0; i < getNumberOfFeatures(); ++i) {

        double area = 0;
        unsigned k = getFeaturePointCount(i) - 1;
        Area_Points temp;
        t_point tempPoint;
        LatLon p1;

        // area = area + (lon_to_x(getFeaturePoint(i, j)) * lat_to_y(getFeaturePoint(i, k)))
        //       -(lon_to_x(getFeaturePoint(i, k)) * lat_to_y(getFeaturePoint(i, j)));
        //k = j;

        for (unsigned j = 0; j < getFeaturePointCount(i); ++j) {

            p1 = getFeaturePoint(i, j);
            area = area + (lon_to_x(getFeaturePoint(i, j)) * lat_to_y(getFeaturePoint(i, k)))
                    -(lon_to_x(getFeaturePoint(i, k)) * lat_to_y(getFeaturePoint(i, j)));
            k = j;
            tempPoint = t_point(lon_to_x(p1), lat_to_y(p1));
            temp.featurePoints.push_back(tempPoint);
        }

        //to avoid negative areas we take the absolute value of the area
        area = abs(area)*0.5;

        if (getFeatureName(i) == "<big ocean>") area = 100000000;
        temp.Featurearea = area;

        featureArea.push_back(std::make_pair(i, temp));
    }
    return featureArea;
}

/*  This function is used to compare feature area, it is used in std::sort for 
 * a custom sort 
 *  @param Temp1 and Temp2: These are two pairs which contain featureID and feature area
 *  @return true or false depending on whether or not the area of the second feature 
 greater than the first*/
bool sort_by_area(std::pair<FeatureIndex, Area_Points> temp1, std::pair<FeatureIndex, Area_Points> temp2) {
    return temp1.second.Featurearea > temp2.second.Featurearea;
}

/*  This function is used to convert a longitude value into a x value using a
 * formula provided documents provided by the school 
 *  @param t: this a the LatLon point that needs to be converted 
 *  @return double: it returns the x-value of the given LatLon point*/
double lon_to_x(LatLon t) {
    return t.lon() * std::cos(average_lat) * DEG_TO_RAD;
}

/*  This function is used to convert a latitude value into a y value using a
 * formula provided documents provided by the school 
 *  @param t: this a the LatLon point that needs to be converted 
 *  @return double: it returns the y-value of the given LatLon point*/
double lat_to_y(LatLon t) {
    return t.lat() * DEG_TO_RAD;
}

/*  This function is used to convert a point(x,y) into a LatLon object by using a
 * formula provided documents provided by the school 
 *  @param (x,y): This is the point to be converted
 *  @return LatLon: The LatLon point corresponding to the point(x,y)*/
LatLon to_spherical(float x, float y) {
    double d = 1 / DEG_TO_RAD;
    double r = 1 / cos(average_lat);
    LatLon t(y*d, x * r * d);
    return t;
}

/*  This function stores the name of the last map opened by the user in case the user
 * doesn't enter a valid map name 
 *  @param stored_map: This is the name of the current map
 *  @return None, because the function doesn't need to return anything*/
void last_map_name(std::string stored_map) {
    last_map = stored_map;
}

/*  This function zooms the map into a specific point (used when user searches for
 * something ect...)
 *  @param (x,y): This is the point to be focused on 
 *  @param zoom_level: how zoomed in or out the screen will be depends on which
 * context the function is used 
 *  @return None, because the function doesn't need to return anything*/
void focus_level(double x, double y, double zoom_level) {
    set_visible_world(x - zoom_level, y - zoom_level, x + zoom_level, y + zoom_level);
}

/*  Function that actually calculates where to focus the screen on and how much to 
 * zoom into or out of the screen
 *  @param None, function uses global variables or internal variables  
 *  @return None, because the function doesn't need to return anything*/
void determine_screen() {
    average_lat = average_lat_again;
    double x = (min_lon * std::cos(average_lat) * DEG_TO_RAD) +
            (max_lon * std::cos(average_lat) * DEG_TO_RAD);
    double y = (min_lat * DEG_TO_RAD) + (max_lat * DEG_TO_RAD);
    focus_level(x * 0.5, y * 0.5 - 0.0007, 0.0015);
}

/*  This function draws a line between two LatLon points
 *  @param p1 and p2: These are the points in between which the line is to be drawn
 *  @return None, because the function doesn't need to return anything*/
void draw_street_segment(LatLon p1, LatLon p2) {
    float x1 = lon_to_x(p1);
    float y1 = lat_to_y(p1);
    float x2 = lon_to_x(p2);
    float y2 = lat_to_y(p2);
    drawline(x1, y1, x2, y2);
}

/*  This takes in a latlon point and a name, it then displays the correct
 * coordinates of the point and displays the name associated with it  
 *  @param position: This is the LatLon point to be displayed
 *  @param name: This is the name to be displayed
 *  @return None, because the function doesn't need to return anything*/
void display_name_and_coord(LatLon position, std::string name) {
    keep_message = true;
    if (position.lat() >= 0 && position.lon() <= 0) {
        std::string message = name + " - Lat: " +
                std::to_string(std::abs(position.lat())) + "N " + " Lon: " +
                std::to_string(std::abs(position.lon())) + "W";
        update_message(message, 0);
    } else if (position.lat() <= 0 && position.lon() <= 0) {
        std::string message = name + " - Lat: " +
                std::to_string(std::abs(position.lat())) + "S " + " Lon: " +
                std::to_string(std::abs(position.lon())) + "W";
        update_message(message, 0);
    } else if (position.lat() >= 0 && position.lon() >= 0) {
        std::string message = name + " - Lat: " +
                std::to_string(std::abs(position.lat())) + "N " + " Lon: " +
                std::to_string(std::abs(position.lon())) + "E";
        update_message(message, 0);
    } else if (position.lat() <= 0 && position.lon() >= 0) {
        std::string message = name + " - Lat: " +
                std::to_string(std::abs(position.lat())) + "S " + " Lon: " +
                std::to_string(std::abs(position.lon())) + "E";
        update_message(message, 0);
    }
}

/**
 * The function
 * @param x_coord
 * @param y_coord
 */

void find_closest_street_segment(double x_coord, double y_coord) {
    //TODO for finding the street segments, could be hard because the
    //center point for streets can be in the middle
    //implementing searching from intersections or streets
    segments_RTree.query(bgi::nearest(point(x_coord, y_coord), 1),
            std::back_inserter(found_street_segments));
    return;
}

/** Creates the buttons for the menu
 *  Possible expansion to more buttons
 * @param void - no inputs needed
 * @return void - no outputs given
 */
void draw_buttons() {
    if (MWIDTH != 0) {

        /*create_button("Zoom Fit", "Find", find_function);
        create_button("Find", "Closest POI", closest_POI_search);
        create_button("Closest POI", "Search Help", search_help);
        create_button("Search Help", "Change Map", change_map);*/

        create_button("Zoom Fit", "Naloxone", closest_POI_search);
        create_button("Naloxone", "Search Help", search_help);
        create_button("Search Help", "Change Map", change_map);
    }
}

/*  This function takes fixes user input (capitalization) so that it is valid for
 * our other functions  
 *  @param to_fix: User string which might be entered incorrectly 
 *  @return to_fix: Same string as above but now with fixed capitalization*/
std::string fix_string(std::string to_fix) {
    to_fix = boost::algorithm::to_lower_copy(to_fix);
    to_fix = capitalizing(to_fix);
    boost::algorithm::trim(to_fix);
    return to_fix;
}

/*  This is a helper function for fix_string it takes in user string and capitalizes 
 *  the first latter of each word
 *  @param text: User string which might be entered incorrectly 
 *  @return string: Same string as above but now with fixed capitalization*/
std::string capitalizing(std::string text) {
    text[0] = std::toupper(text[0]);
    std::transform(text.begin() + 1, text.end(), text.begin(), text.begin() + 1,
            [](const char& a, const char& b) -> char {
                if (b == ' ' || b == '\t') return toupper(a);
                return a;
            });
    return text;
}

/*  This function is used to change the current map to another city
 *  @param void(*draw_map): This is an input to the function it is required to 
 *      properly define the map to be loaded 
 *  @return None, because the function doesn't need to return anything*/
void change_map(void (*draw_map) ()) {
    /*closes current graphics window, gets input from user and if the map exists
     * it opens the graphics window again with the new map loaded
     * otherwise it opens back to where the last map was left off at
     */

    change_map_select = true;
    opened_found_path = false;
    close_tab();
    found_search.clear();
    keep = false;
    has_opened = false;
    keep_message = false;
    opened_found_path = false;
    reset_highlights();
    saved_POI.clear();
    saved_intersections.clear();
    selected_POI = false;
    selected_intersection = false;
    path_at = 0;

    std::cout << "Type in the city country you want to change to, eg. cape-town south-africa: ";
    std::string map_name = "/cad2/ece297s/public/maps/";
    std::string city, country, temp_name;
    getline(std::cin, temp_name);
    if (boost::algorithm::to_lower_copy(temp_name) == "saint-helena" ||
            boost::algorithm::to_lower_copy(temp_name) == "iceland" ||
            boost::algorithm::to_lower_copy(temp_name) == "singapore") {
        map_name = map_name + temp_name + ".streets.bin";
    } else {
        std::stringstream ss(temp_name);
        ss >> city >> country;

        map_name = map_name + boost::algorithm::to_lower_copy(city) + "_" +
                boost::algorithm::to_lower_copy(country) + ".streets.bin";
    }
    close_map();
    //closeOSMDatabase();
    featuresArea.clear();
    if (load_map(map_name)) {
        std::size_t pos = map_name.find("streets.bin");
        std::string osm_bin_name = map_name.replace(pos, map_name.length(), "osm.bin");
        //loadOSMDatabaseBIN(osm_bin_name);
        open_tab();
        clearscreen();
        initalize_map();
        determine_screen();
        keep_message = true;
        update_message("Success", 0);
    } else {
        load_map(last_map);
        std::size_t pos = last_map.find("streets.bin");
        std::string osm_bin_name = last_map.replace(pos, last_map.length(), "osm.bin");
        //loadOSMDatabaseBIN(osm_bin_name);
        open_tab();
        initalize_map();
        keep_message = true;
        update_message("Unsuccessful", 0);
    }

    (*draw_map) ();


}

void search_help(void (*draw_screen) ()) {
    keep_message = true;
    update_message("Type 2 streets into search bar = Intersection search, POI name = POI search, Street name = street search: Ex. college street & spadina avenue then click and drag your mouse to continue"); // can clear the message and directly write the text here
    (*draw_screen) ();
}

void closest_POI_search(void (*draw_screen) ()) {
    do_closest_POI_search = true;
    if (saved_intersections.size() == 1) {
        keep_message = true;
        update_message("You already have a starting intersection, now type your desired Point of Interest");
    } else {
        keep = false;
        keep_message = true;
        do_closest_POI_search = true;
        update_message("Type your desired Point of Interest");
    }
    (*draw_screen) ();
}

/*  This function is used to highlight an intersection when a user hovers over it
 * and to display information about the intersection
 *  @param (x,y): These are the the current mouse coordinates
 *  @return None, because the function doesn't need to return anything*/
void act_on_mousemove(float x, float y) {
    LatLon mouse_position = to_spherical(x, y);
    unsigned id = find_closest_intersection(mouse_position);
    if (LOD_area_test(0.0000001) && y >= lat_to_y(getIntersectionPosition(id)) - SMALL_BOX_RANGE &&
            x >= lon_to_x(getIntersectionPosition(id)) - SMALL_BOX_RANGE &&
            y <= lat_to_y(getIntersectionPosition(id)) + SMALL_BOX_RANGE &&
            x <= lon_to_x(getIntersectionPosition(id)) + SMALL_BOX_RANGE) {

        setcolor(235, 0, 0, 200);
        found_intersect(true);
        draw_screen();
        //draw_intersection_highlight(lon_to_x(getIntersectionPosition(id)),
        //                            lat_to_y(getIntersectionPosition(id)));
        if (!opened_found_path)
            display_name_and_coord(getIntersectionPosition(id), getIntersectionName(id));

    } else {
        id = find_closest_point_of_interest(mouse_position);
        if (LOD_area_test(0.0000001) &&
                y >= lat_to_y(getPointOfInterestPosition(id)) - SMALL_BOX_RANGE &&
                x >= lon_to_x(getPointOfInterestPosition(id)) - SMALL_BOX_RANGE &&
                y <= lat_to_y(getPointOfInterestPosition(id)) + SMALL_BOX_RANGE &&
                x <= lon_to_x(getPointOfInterestPosition(id)) + SMALL_BOX_RANGE) {
            if (!opened_found_path)
                update_message(getPointOfInterestName(id), 0);
        } else {
            found_intersect(false);
            if (!keep_message && !opened_found_path)
                update_message("", 0);
        }
    }
}

/*  This function is used to draw and print the name of certain POI depending on
 * the zoom level of the screen, it looks at POI type and then chooses which icon to
 * display on the map
 *  @param area: Area of the current visible screen being displayed, used as zoom level
 *  @param index: POI_ID
 *  @param POI_to_draw: vector of all POI visible on current screen 
 *  @return None, because the function doesn't need to return anything*/
void draw_POI(double area, int index, std::vector<value> POI_to_draw) {

    LatLon pos = getPointOfInterestPosition(POI_to_draw[index].second);
    if (LOD_area_test(area)) {
        if (getPointOfInterestType(POI_to_draw[index].second) == "fast_food") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/001-food.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "restaurant") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/035-dinner.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "cafe") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/034-coffee-cup.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "bank") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/010-bank-building.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "fuel") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/031-gas-station.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "pharmacy") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/007-tablet-and-pill.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "pub") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/002-beer.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "place_of_worship") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/033-muslim-praying.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "doctors") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/008-doctor-stethoscope.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "school") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/039-open-book.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "vending_machine") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/029-vending-machine.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "parking") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/013-parking-sign.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "library") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/039-open-book.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "post_office") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/025-mailbox.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "bar") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/002-beer.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else if (getPointOfInterestType(POI_to_draw[index].second) == "atm") {
            Surface toDraw = load_png_from_file("libstreetmap/resources/027-atm.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        } else {
            Surface toDraw = load_png_from_file("libstreetmap/resources/pin.png");
            draw_surface(toDraw, lon_to_x(pos), lat_to_y(pos), 0);
        }
    }

    if (LOD_area_test(area * 0.1)) {

    }
}

/*  This function is used to draw the screen,it is repeatedly being called in draw_map
 *  @param none: This function does not required any parameters because it takes deals with
 * other function and global variables  
 *  @return None, because the function doesn't need to return anything*/
void draw_screen() {



    clearscreen();
    set_drawing_buffer(OFF_SCREEN);
    clearscreen();
    //setcolor(235, 235, 235, 255);
    //fillrect(min_lon * std::cos(average_lat) * DEG_TO_RAD, min_lat*DEG_TO_RAD, max_lon * std::cos(average_lat) * DEG_TO_RAD, max_lat * DEG_TO_RAD);
    flushinput();
    t_bound_box b = get_visible_world();
    box search_box(point(b.left() - 0.000001, b.bottom() - 0.000001),
            point(b.right() + 0.000001, b.top() + 0.000001));

    std::vector<value> features_to_check;
    f_rtree.query(!bgi::disjoint(search_box), std::back_inserter(features_to_check));
    std::vector<value> streetIDs_to_check;
    segments_RTree.query(!bgi::disjoint(search_box), std::back_inserter(streetIDs_to_check));
    std::vector<value> POI_to_draw;
    POI_RTree.query(!bgi::disjoint(search_box), std::back_inserter(POI_to_draw));


    /*iterates through featuresArea (sorted vector or features area)
     *checks to see what feature type it is and if it is a closed  feature or not
     * then draws either a line or a polygon and fills it up 
     */
    std::vector<std::pair<FeatureIndex, Area_Points>> sorted;

    for (unsigned i = 0; i < features_to_check.size(); ++i) {
        sorted.push_back(featuresArea[features_to_check[i].second]);
    }
    std::sort(sorted.begin(), sorted.end(), sort_by_area);

    for (unsigned i = 0; i < sorted.size(); ++i) {

        /*if(i!= 0 && sorted[i].second == sorted[i-1].second){
            std::cout << getFeatureName(sorted[i-1].first) << "\n";
        }*/


        bool draw_feature = false;

        FeatureType type = getFeatureType(sorted[i].first);
        if (type == Park && LOD_area_test(0.1)) {
            setcolor(150, 255, 75, 255);
            draw_feature = true;
        } else if (type == Beach && LOD_area_test(0.01)) {
            setcolor(255, 255, 204, 255);
            draw_feature = true;
        } else if (type == Lake && LOD_area_test(0.01)) {
            setcolor(0, 76, 153, 255);
            draw_feature = true;
        } else if (type == River && LOD_area_test(0.01)) {
            setcolor(0, 76, 153, 255);
            draw_feature = true;
        } else if (type == Island && LOD_area_test(0.01)) {
            setcolor(235, 235, 235, 255);
            draw_feature = true;
        } else if (type == Shoreline && LOD_area_test(0.0001)) {
            setcolor(0, 76, 153, 255);
            draw_feature = true;
        } else if (type == Building && LOD_area_test(0.0000001)) {
            setcolor(204, 204, 204, 255);
            draw_feature = true;
        } else if (type == Greenspace && LOD_area_test(0.00001)) {
            setcolor(150, 255, 75, 255);
            draw_feature = true;
        } else if (type == Golfcourse && LOD_area_test(0.00001)) {
            setcolor(150, 255, 75, 255);
            draw_feature = true;
        } else if (type == Stream && LOD_area_test(0.00001)) {
            setcolor(0, 76, 153, 255);
            draw_feature = true;
        } else if (type == Unknown && LOD_area_test(0.000001)) {
            setcolor(0, 0, 0, 255);
            draw_feature = true;
        } else {
            setcolor(235, 235, 235, 255);

        }
        if (draw_feature == true) {
            //t_point features_array[getFeaturePointCount(sorted[i].first)];
            //t_point features_array[getFeaturePointCount(sorted[i].first)];

            t_point* features_array = &sorted[i].second.featurePoints[0];
            std::copy(sorted[i].second.featurePoints.begin(), sorted[i].second.featurePoints.end(), features_array);
            setlinewidth(3);
            LatLon p1;
            LatLon p2;
            for (unsigned j = 0; j < getFeaturePointCount(sorted[i].first) - 1; ++j) {

                p1 = getFeaturePoint(sorted[i].first, j);
                p2 = getFeaturePoint(sorted[i].first, j + 1);
                float x1 = lon_to_x(p1);
                float y1 = lat_to_y(p1);
                //volatile float x2 = features_array[j+1].x;
                //volatile float y2 = features_array[j+1].y;
                //unsigned f1 = sorted[i].first;
                float x2 = lon_to_x(p2);
                float y2 = lat_to_y(p2);
                features_array[j] = t_point(x1, y1);

                drawline(x1, y1, x2, y2);
            }

            unsigned f = getFeaturePointCount(sorted[i].first) - 1;

            features_array[f] = t_point(lon_to_x(getFeaturePoint(sorted[i].first, f)), lat_to_y(getFeaturePoint(sorted[i].first, f)));
            /*getFeaturePoint(sorted[i].first, 0).lat() == getFeaturePoint(sorted[i].first, f).lat() &&
                    getFeaturePoint(sorted[i].first, 0).lon() == getFeaturePoint(sorted[i].first, f).lon()*/

            if (getFeaturePoint(sorted[i].first, 0).lat() == getFeaturePoint(sorted[i].first, f).lat() &&
                    getFeaturePoint(sorted[i].first, 0).lon() == getFeaturePoint(sorted[i].first, f).lon()) {
                fillpoly(features_array, getFeaturePointCount(sorted[i].first));

            }
            setfontsize(FONT_SIZE_12);
            setcolor(0, 0, 0, 255);

        }
    }

    //Used to find all things within screen range

    //Displaying names and direction onto the map
    for (unsigned i = 0; i < streetIDs_to_check.size(); ++i) {
        unsigned streetSegID = streetIDs_to_check[i].second;
        StreetSegmentInfo s = getStreetSegmentInfo(streetSegID);
        std::string street_type = street_classifier(s);
        bool draw_current_segment = draw_segment(street_type);
        //Drawing direction and name onto the screen
        if (draw_current_segment == true) {
            int curve_pts = s.curvePointCount;
            LatLon p1;
            LatLon p2;


            if (curve_pts == 0) {
                setlinestyle(0, 1); //Changing to straight
                p1 = getIntersectionPosition(s.from);
                p2 = getIntersectionPosition(s.to);
            } else {
                int j = 0; //Index of curve point
                for (j = 0; j < curve_pts; ++j) {
                    //Go through all curve points and draw each with their respective conditions
                    if (j == 0) {
                        setlinestyle(0, 1); //Changing to straight
                        p1 = getIntersectionPosition(s.from);
                        p2 = getStreetSegmentCurvePoint(streetSegID, j);

                    } else {
                        setlinestyle(0, 1); //Changing to curved
                        p1 = getStreetSegmentCurvePoint(streetSegID, j - 1);
                        p2 = getStreetSegmentCurvePoint(streetSegID, j);
                    }
                    if (draw_current_segment == true) {
                        //Finding distances for labels to place
                        //double d = find_distance_between_two_points(p1, p2);

                        draw_street_segment(p1, p2);
                    }
                }
                p2 = getIntersectionPosition(s.to);
                p1 = getStreetSegmentCurvePoint(streetSegID, j - 1);
            }
            if (draw_current_segment == true) {
                draw_street_segment(p1, p2);
            }
            //Street name label

        }
    }


    for (unsigned i = 0; i < streetIDs_to_check.size(); i++) {
        unsigned streetSegID = streetIDs_to_check[i].second;
        StreetSegmentInfo s = getStreetSegmentInfo(streetSegID);
        std::string street_type = street_classifier(s);
        bool draw_current_segment = draw_segment(street_type);
        int curve_pts = s.curvePointCount;

        if (LOD_area_test(0.000001) && draw_current_segment == true) {
            LatLon p;
            double x;
            double y;
            LatLon pt1;
            LatLon pt2;
            bool oneway = s.oneWay;
            std::string street_name = getStreetName(s.streetID);
            //Avoid showing unknown but want to show arrow
            if (street_name == "<unknown>") {
                street_name = "";
            }
            double angle;
            bool draw_current_label = true;
            if (curve_pts == 0) {
                pt1 = getIntersectionPosition(s.from);
                pt2 = getIntersectionPosition(s.to);
                x = (pt1.lon() + pt2.lon())*0.5;
                y = (pt1.lat() + pt2.lat())*0.5;
                p = LatLon(y, x);
                double RAD_TO_DEG = 1 / DEG_TO_RAD;
                angle = atan2(lat_to_y(pt2) - lat_to_y(pt1), lon_to_x(pt2) - lon_to_x(pt1)) * RAD_TO_DEG;
                if (angle > 90) {
                    if (oneway == true) {
                        street_name = "<- " + street_name;
                    }
                    angle = angle - 180;
                } else if (angle < -90) {
                    if (oneway == true) {
                        street_name = "<- " + street_name;
                    }
                    angle = angle + 180;
                } else {
                    if (oneway == true) {
                        street_name = street_name + " ->";
                    }
                }


            } else {
                int curve_pt_to_label = 0;
                int j;
                double distance = 0;
                LatLon p1, p2;
                for (j = 0; j < curve_pts; ++j) {
                    //Go through all curve points and draw each with their respective conditions
                    if (j == 0) {
                        setlinestyle(0, 1); //Changing to straight
                        p1 = getIntersectionPosition(s.from);
                        p2 = getStreetSegmentCurvePoint(streetSegID, j);
                        double d = find_distance_between_two_points(p1, p2);
                        if (d > distance) {
                            curve_pt_to_label = 0;
                            distance = d;
                        }

                    } else {
                        setlinestyle(0, 1); //Changing to curved
                        p1 = getStreetSegmentCurvePoint(streetSegID, j - 1);
                        p2 = getStreetSegmentCurvePoint(streetSegID, j);
                        double d = find_distance_between_two_points(p1, p2);
                        if (d > distance) {
                            curve_pt_to_label = j;
                            distance = d;
                        }
                    }
                }
                p1 = getStreetSegmentCurvePoint(streetSegID, j - 1);
                p2 = getIntersectionPosition(s.to);
                if (find_distance_between_two_points(p1, p2) > distance) {
                    curve_pt_to_label = -1;
                }
                if (curve_pt_to_label == -1) {
                    pt1 = getStreetSegmentCurvePoint(streetSegID, curve_pts - 1);
                    pt2 = getIntersectionPosition(s.to);
                } else if (curve_pt_to_label == 0) {
                    pt1 = getIntersectionPosition(s.from);
                    pt2 = getStreetSegmentCurvePoint(streetSegID, curve_pt_to_label);
                } else {
                    pt1 = getStreetSegmentCurvePoint(streetSegID, curve_pt_to_label - 1);
                    pt2 = getStreetSegmentCurvePoint(streetSegID, curve_pt_to_label);
                }
                x = (pt1.lon() + pt2.lon())*0.5;
                y = (pt1.lat() + pt2.lat())*0.5;
                p = LatLon(y, x);
                angle = atan2(lat_to_y(pt2) - lat_to_y(pt1), lon_to_x(pt2) - lon_to_x(pt1)) / DEG_TO_RAD;
                //Displaying text, getting and fixing angle to draw slanted
                if (angle > 90) {
                    if (oneway == true) {
                        street_name = "<- " + street_name;
                    }
                    angle = angle - 180;

                } else if (angle < -90) {
                    if (oneway == true) {
                        street_name = "<- " + street_name;
                    }
                    angle = angle + 180;
                } else {
                    if (oneway == true) {
                        street_name = street_name + " ->";
                    }
                }
            }
            x = lon_to_x(p);
            y = lat_to_y(p);
            double sdistance = sqrt(pow(lon_to_x(pt2) - lon_to_x(pt1), 2) +
                    pow(lat_to_y(pt2) - lat_to_y(pt1), 2));
            settextrotation(angle);
            setcolor(0, 0, 0, 255);

            //For specific specifications of highways
            if (street_type == "Highway" && draw_current_label == true) {
                setfontsize(FONT_SIZE_18); //upper bound
                drawtext(x, y, street_name, FLT_MAX, FLT_MAX, true, sdistance, 14);
            } else if (street_type == "Big Street" && draw_current_label == true) {
                drawtext(x, y, street_name, FLT_MAX, FLT_MAX, true, sdistance, 9);
            } else if (street_type == "Small Street" && LOD_area_test(0.00000007)
                    && draw_current_label == true) {
                setfontsize(FONT_SIZE_10); //upper-bound for font size of small streets
                drawtext(x, y, street_name, FLT_MAX, FLT_MAX, true, sdistance, 7); //default lower bound is set to 6
            }
            //back to default settings
            settextrotation(0);
            setfontsize(FONT_SIZE_16);
            setcolor(0, 0, 0, 255);
        }
    }

    /*Uses R-tree to check which POI are on the visible screen currently and sotres them in
     * a vector, iterates through that vector and calls function draw_POI multiple times to 
     * draw POI in more or less detail depending on zoom level
     */



    if (LOD_area_test(0.0000000035)) {
        for (unsigned i = 0; i < POI_to_draw.size(); ++i) {
            LatLon pos = getPointOfInterestPosition(POI_to_draw[i].second);
            t_point topRight = t_point(lon_to_x(pos) + 0.0000005, lat_to_y(pos) + 0.0000005);
            t_point BottomLeft = t_point(lon_to_x(pos) - 0.0000005, lat_to_y(pos) - 0.0000005);
            t_bound_box POI_box = t_bound_box(BottomLeft, topRight);
            for (unsigned j = 0; j < POI_to_draw.size(); ++j) {
                LatLon posTwo = getPointOfInterestPosition(POI_to_draw[j].second);
                bool draw = POI_box.intersects(lon_to_x(posTwo), lat_to_y(posTwo));
                if (j != i && !draw) {
                    draw_POI(0.00000001, i, POI_to_draw);
                }
                if (j != i && draw == false) {
                    draw_POI(0.0000000035, i, POI_to_draw);
                }

            }
        }
    }

    if (shouldFind) {


        double average_x = 0;
        double average_y = 0;
        int num = 0;
        if (search_method != STREETS_SEARCH) {
            for (std::vector<unsigned>::iterator iter = found_search.begin();
                    iter != found_search.end(); ++iter) {
                LatLon position_of_object;
                if (search_method == INTERSECTION_SEARCH) {
                    position_of_object = getIntersectionPosition(*iter);
                } else if (search_method == POI_SEARCH) {
                    position_of_object = getPointOfInterestPosition(*iter);
                }
                force_setcolor(t_color(255, 0, 0, 255));
                if (saved_intersections.size() != 1) {
                    fillarc(lon_to_x(position_of_object), lat_to_y(position_of_object),
                            RADIUS_OF_CIRCLE, 0, 360);
                }

                average_x = average_x + lon_to_x(position_of_object);
                average_y = average_y + lat_to_y(position_of_object);
                flushinput();
                ++num;
            }
        } else if (search_method == STREETS_SEARCH) {
            //Iterate through the vector of ids, by finding all street segments and
            //curve points and highlighting those specific ones needed
            draw_street_highlight(found_search, average_x, average_y, num);
        }
        //If no points found then set it to 1
        if (num <= 0) {
            num = 1;
        }
        //Keeping focus on that location until keep is false
        if (keep && search_method != POI_SEARCH) {

            focus_level(average_x / num, average_y / num, 0.0001);
            flushinput();
        }

    }

    if (selected_intersection || selected_POI) {

        double average_x = 0, average_y = 0;
        int num = 0;
        if (draw_intersection_path || draw_POI_path || draw_intersection_to_POI_path ||
                draw_POI_to_intersection_path || draw_closest_POI_to_intersection_path) {

            if (draw_intersection_path) {

                draw_street_segment_highlight(found_intersection_path, average_x, average_y, num);
                //update_message(directions[i], 2);

            } else if (draw_POI_path) {

                draw_street_segment_highlight(found_POIs_path, average_x, average_y, num);
                //update_message(directions[i], 2);

            } else if (draw_intersection_to_POI_path) {

                draw_street_segment_highlight(found_intersection_to_POI_path, average_x, average_y, num);
                //update_message(directions[i], 2);

            } else if (draw_POI_to_intersection_path) {

                draw_street_segment_highlight(found_POI_to_intersection_path, average_x, average_y, num);

                //update_message(directions[i], 2);

            } else if (draw_closest_POI_to_intersection_path) {

                unsigned id = closest_POI;
                LatLon POI_position = getPointOfInterestPosition(id);
                draw_surface(POI_pin, lon_to_x(POI_position),
                        lat_to_y(POI_position), 3);
                draw_street_segment_highlight(found_closest_POI_to_intersection_path, average_x, average_y, num);
                //update_message(directions[i], 2);

            }
            if (MWIDTH > 0)
                draw_surface(close_menu_arrow, get_visible_screen().right() - 145,
                    get_visible_screen().bottom() - 27, 1);
            else
                draw_surface(close_menu_arrow, get_visible_screen().right() - 40,
                    get_visible_screen().bottom() - 27, 1);

            draw_surface(open_menu_arrow, 0, get_visible_screen().bottom() - 27, 1);
        }
        if (selected_intersection) {
            for (unsigned i = 0; i < saved_intersections.size(); ++i) {
                unsigned id = saved_intersections[i];
                LatLon position = getIntersectionPosition(id);
                draw_surface(intersection_pin, lon_to_x(position), lat_to_y(position), 2);
            }
        }
        if (selected_POI) {
            for (unsigned j = 0; j < saved_POI.size(); ++j) {
                unsigned id = saved_POI[j];
                LatLon position = getPointOfInterestPosition(id);
                draw_surface(POI_pin, lon_to_x(position), lat_to_y(position), 3);
            }
        }


    }

    if (MWIDTH > 0) {
        draw_surface(close_menu_arrow, get_visible_screen().right() - 145, 10, 1);
        //draw_surface(close_menu_arrow, 672, 10, 1);
    } else {
        draw_surface(open_menu_arrow, get_visible_screen().right() - 40, 10, 1);
        //draw_surface(open_menu_arrow, 780, 10, 1);
    }

    //show to screen, buffer stuff and clear vectors
    flushinput();
    copy_off_screen_buffer_to_screen();
    clearscreen();
    POI_to_draw.clear();
    streetIDs_to_check.clear();




}

/*  This function is used to classify streets segments into minor/major roads depending on the
 * speed limit
 *  @param s: streetSegmentInfo for classification 
 *  @return string: This string classifies the streetSegment using speedLimits*/
std::string street_classifier(StreetSegmentInfo s) {

    std::string OSMValue = getTagPair(OSMWayStreets[s.wayOSMID].first, OSMWayStreets[s.wayOSMID].second).second;

    if (OSMValue == "motorway" || OSMValue == "trunk" || OSMValue == "motorway_link" || OSMValue == "trunk_link") {
        return "Highway";
    } else if (OSMValue == "primary" || OSMValue == "secondary" || OSMValue == "primary_link" || OSMValue == "secondary_link") {
        return "Big Street";
    } else {
        return "Small Street";
    }

    /*if (s.speedLimit >= HIGHWAY_SPEED_LIMIT) {
        return "Highway";
    } else if (s.speedLimit <= BIG_STREET_UPPER_LIMIT &&
            s.speedLimit > BIG_STREET_LOWER_LIMIT) {
        return "Big Street";
    } else {

        return "Small Street";
    }*/
}

/** Classifies the street
 * @param street_type the street's type being highway, or big/small street
 * @return tells the draw screen function to draw the street
 */
bool draw_segment(std::string street_type) {
    bool draw_current_segment = true;
    if (street_type == "Highway") {
        setcolor(210, 105, 30, 255);
        setlinewidth(12);
        //At this lOD area change size
        if (LOD_area_test(0.000000005)) {
            setlinewidth(21);
        }
    } else if (street_type == "Big Street" && LOD_area_test(0.00002)) {
        setcolor(255, 255, 155, 255);
        setlinewidth(5);
        //At this lOD area change size
        if (LOD_area_test(0.000000005)) {
            setlinewidth(18);
        }
    } else if (street_type == "Small Street") {
        setcolor(255, 255, 255, 255);
        setlinewidth(3);
        //At this lOD area change size
        if (LOD_area_test(0.000000005)) {
            setlinewidth(15);
        }
        //At this lOD area stop drawing
        if (!LOD_area_test(0.000001)) {
            draw_current_segment = false;
        }
    }
    return draw_current_segment;
}

/*  This function is used to preform an operation depending on what mouse button is
 * pressed, function is repeatedly called in event loop
 *  @param (x,y): This point is the coordinates of the mouse 
 *  @param (event): Structure which is used to check if the mouse button is clicked or not
 *  @return None, because the function doesn't need to return anything*/
void act_on_button_press(float x, float y, t_event_buttonPressed event) {

    if (event.button == Button1) { //Left mouse button is pressed
        do_closest_POI_search = false;
        keep = false;
        //Intersection to intersection
        unsigned id = find_closest_intersection(to_spherical(x, y));
        LatLon position = getIntersectionPosition(id);
        double point_x = lon_to_x(position);
        double point_y = lat_to_y(position);

        if (LOD_area_test(0.000001) && ((point_x <= x + SMALL_BOX_RANGE &&
                point_x >= x - SMALL_BOX_RANGE) && (point_y <= y +
                SMALL_BOX_RANGE && point_y >= y - SMALL_BOX_RANGE))) {
            display_name_and_coord(position, getIntersectionName(id));

            if (draw_POI_to_intersection_path || draw_intersection_to_POI_path ||
                    draw_POI_path) {
                saved_intersections.clear();
                saved_POI.clear();
            }
            reset_highlights();


            if (saved_intersections.size() < 2) {
                saved_intersections.push_back(id);
                selected_intersection = true;

                if (saved_intersections.size() == 1) {
                    keep_message = true;
                    update_message("Click on another intersection, POI or type a desired POI");
                }

                if (saved_intersections.size() == 1 && saved_POI.size() == 1) {
                    selected_POI = true;
                    LatLon POI_position = getPointOfInterestPosition(saved_POI[0]);
                    unsigned closest_intersect = find_closest_intersection(POI_position);
                    saved_intersection_to_POI.push_back(closest_intersect);
                    saved_intersection_to_POI.push_back(saved_intersections[0]);

                    found_POI_to_intersection_path = find_path_between_intersections(
                            saved_intersection_to_POI[0],
                            saved_intersection_to_POI[1],
                            TURN_PENALTY);

                    opened_found_path = true;
                    directions = all_turns(found_POI_to_intersection_path);
                    max_segments = directions.size();
                    has_opened = false;
                    path_at = 0;
                    if (max_segments == 0) {
                        update_message("Either you have arrived at your destination, or there is no valid path", 0);
                    } else update_message(directions[path_at], 0);
                    draw_POI_to_intersection_path = true;

                } else if (saved_intersections.size() == 2) {
                    saved_POI.clear();
                    selected_POI = false;
                    selected_intersection = true;
                    found_intersection_path =
                            find_path_between_intersections(saved_intersections[0],
                            saved_intersections[1],
                            TURN_PENALTY);

                    opened_found_path = true;
                    directions = all_turns(found_intersection_path);
                    max_segments = directions.size();
                    path_at = 0;
                    has_opened = false;
                    if (max_segments == 0) {
                        update_message("Either you have arrived at your destination, or there is no valid path", 0);
                    } else update_message(directions[path_at], 0);
                    draw_intersection_path = true;
                }

            } else {
                saved_intersections.clear();
                saved_intersections.push_back(id);
                selected_intersection = true;
                draw_intersection_path = false;
            }
        }
        //POI to POI
        id = find_closest_point_of_interest(to_spherical(x, y));
        position = getPointOfInterestPosition(id);
        point_x = lon_to_x(position);
        point_y = lat_to_y(position);
        if (LOD_area_test(0.000001) && ((point_x <= x + SMALL_BOX_RANGE &&
                point_x >= x - SMALL_BOX_RANGE) && (point_y <= y +
                SMALL_BOX_RANGE && point_y >= y - SMALL_BOX_RANGE))) {
            if (draw_POI_to_intersection_path || draw_intersection_to_POI_path
                    || draw_intersection_path) {
                saved_intersections.clear();
                saved_POI.clear();
            }
            reset_highlights();

            if (saved_POI.size() < 2) {
                selected_POI = true;
                saved_POI.push_back(id);
                if (saved_POI.size() == 1) {
                    keep_message = true;
                    update_message("Click on another POI or intersection to search for a path");
                }
                draw_intersection_to_POI_path = false;
                draw_POI_path = false;
                if (saved_intersections.size() == 1 && saved_POI.size() == 1) {

                    LatLon POI_position = getPointOfInterestPosition(saved_POI[0]);
                    unsigned closest_intersect = find_closest_intersection(POI_position);
                    saved_POI_to_intersection.push_back(saved_intersections[0]);
                    saved_POI_to_intersection.push_back(closest_intersect);
                    found_intersection_to_POI_path = find_path_between_intersections(
                            saved_POI_to_intersection[0],
                            saved_POI_to_intersection[1],
                            TURN_PENALTY);
                    opened_found_path = true;
                    directions = all_turns(found_intersection_to_POI_path);
                    max_segments = directions.size();
                    path_at = 0;
                    has_opened = false;
                    if (max_segments == 0) {
                        update_message("Either you have arrived at your destination, or there is no valid path", 0);
                    } else update_message(directions[path_at], 0);
                    draw_intersection_to_POI_path = true;
                } else if (saved_POI.size() == 2) {
                    saved_intersections.clear();
                    selected_intersection = false;
                    selected_POI = true;

                    for (unsigned i = 0; i < saved_POI.size(); ++i) {
                        saved_intersections.push_back(find_closest_intersection(
                                getPointOfInterestPosition(saved_POI[i])));
                    }
                    found_POIs_path = find_path_between_intersections(saved_intersections[0],
                            saved_intersections[1],
                            TURN_PENALTY);

                    opened_found_path = true;

                    draw_POI_path = true;
                    has_opened = false;
                    directions = all_turns(found_POIs_path);
                    max_segments = directions.size();
                    path_at = 0;
                    if (max_segments == 0) {
                        update_message("Either you have arrived at your destination, or there is no valid path", 0);
                    } else update_message(directions[path_at], 0);


                    saved_intersections.clear();
                }
            } else {
                saved_POI.clear();
                saved_POI.push_back(id);
                draw_POI_path = false;
                selected_POI = true;
            }
        }
    } else if (event.button == Button3) {//Right mouse button is pressed
        keep = false;
        has_opened = false;
        keep_message = false;
        opened_found_path = false;
        reset_highlights();
        saved_POI.clear();
        saved_intersections.clear();
        selected_POI = false;
        selected_intersection = false;
        path_at = 0;
    }
    flushinput();
    draw_screen();
}

void reset_highlights() {
    found_street_segments.clear();
    saved_POI_to_intersection.clear();
    saved_intersection_to_POI.clear();
    found_intersection_path.clear();
    found_POI_to_intersection_path.clear();
    found_intersection_to_POI_path.clear();
    found_closest_POI_to_intersection_path.clear();
    found_POIs_path.clear();
    found_search.clear();
    shouldFind = false;
    draw_POI_path = false;
    do_closest_POI_search = false;
    draw_intersection_path = false;
    draw_POI_to_intersection_path = false;
    draw_intersection_to_POI_path = false;
    draw_closest_POI_to_intersection_path = false;
    search_method = 0;
}

/**Initialize map sets up the basics of the map except setting up the event loop
 * This function will be used by open tab to reinitialize the map to open it again
 * @parm void no parameters needed
 * @return no return needed, everything is automatic
 */
void initalize_map() {
    t_color background = t_color(235, 235, 235, 255);
    featuresArea = sort_feature_by_area();
    sortOSMWaysbyOSMID();
    init_graphics("Map", background);
    draw_buttons();
    found_street_segments.clear();
    saved_intersections.clear();
    found_search.clear();
    determine_screen();
    update_message("Type to begin searches", 1);
}

/* Draw map draws the map and creates its event loop to make map usable
 * @param void nothing needed as input
 * @return void no return needed
 */
void draw_map() {
    initalize_map();
    determine_screen();
    event_loop(act_on_button_press, act_on_mousemove, nullptr, draw_screen);
    close_graphics();
}

void draw_street_highlight(std::vector<unsigned> to_draw, double &average_x, double &average_y, int &num) {
    for (std::vector<unsigned>::iterator iter = to_draw.begin();
            iter != to_draw.end(); ++iter) {
        std::vector<unsigned> ids = find_street_street_segments(*iter);
        for (std::vector<unsigned>::iterator iter_1 = ids.begin();
                iter_1 != ids.end(); ++iter_1) {
            //The rest is the same as the one to draw segments above
            //would make into a function but ran out of time
            StreetSegmentInfo seg = getStreetSegmentInfo(*iter_1);
            LatLon position_of_object;
            setcolor(200, 0, 0, 200);
            LatLon p1, p2;
            if (seg.curvePointCount == 0) {
                p1 = getIntersectionPosition(seg.from);
                p2 = getIntersectionPosition(seg.to);
            } else {
                unsigned j = 0;
                for (j = 0; j < seg.curvePointCount; ++j) {
                    position_of_object = getStreetSegmentCurvePoint(*iter_1, j);
                    if (j == 0) {
                        setlinestyle(0, 1);
                        p1 = getIntersectionPosition(seg.from);
                        p2 = getStreetSegmentCurvePoint(*iter_1, j);
                        draw_street_segment(p1, p2);

                    } else {
                        setlinestyle(0, 1);
                        p1 = getStreetSegmentCurvePoint(*iter_1, j - 1);
                        p2 = getStreetSegmentCurvePoint(*iter_1, j);
                        draw_street_segment(p1, p2);
                    }
                    setlinestyle(0, 1);
                    average_x = average_x + lon_to_x(position_of_object);
                    average_y = average_y + lat_to_y(position_of_object);
                    flushinput();
                    ++num;
                }
                p2 = getIntersectionPosition(seg.to);
                p1 = getStreetSegmentCurvePoint(*iter_1, j - 1);
                draw_street_segment(p1, p2);
                average_x = average_x + lon_to_x(position_of_object);
                average_y = average_y + lat_to_y(position_of_object);
                flushinput();
                ++num;
            }
            draw_street_segment(p1, p2);
        }
    }
}

void draw_street_segment_highlight(std::vector<unsigned> to_draw, double &average_x, double &average_y, int &num) {
    for (std::vector<unsigned>::iterator iter_1 = to_draw.begin();
            iter_1 != to_draw.end(); ++iter_1) {
        //The rest is the same as the one to draw segments above
        //would make into a function but ran out of time
        StreetSegmentInfo seg = getStreetSegmentInfo(*iter_1);
        LatLon position_of_object;
        setcolor(200, 0, 0, 200);
        LatLon p1, p2;
        if (seg.curvePointCount == 0) {
            p1 = getIntersectionPosition(seg.from);
            p2 = getIntersectionPosition(seg.to);
        } else {
            unsigned j = 0;
            for (j = 0; j < seg.curvePointCount; ++j) {
                position_of_object = getStreetSegmentCurvePoint(*iter_1, j);
                if (j == 0) {
                    setlinestyle(0, 1);
                    p1 = getIntersectionPosition(seg.from);
                    p2 = getStreetSegmentCurvePoint(*iter_1, j);
                    draw_street_segment(p1, p2);

                } else {
                    setlinestyle(0, 1);
                    p1 = getStreetSegmentCurvePoint(*iter_1, j - 1);
                    p2 = getStreetSegmentCurvePoint(*iter_1, j);
                    draw_street_segment(p1, p2);
                }
                setlinestyle(0, 1);
                average_x = average_x + lon_to_x(position_of_object);
                average_y = average_y + lat_to_y(position_of_object);
                flushinput();
                ++num;
            }
            p2 = getIntersectionPosition(seg.to);
            p1 = getStreetSegmentCurvePoint(*iter_1, j - 1);
            draw_street_segment(p1, p2);
            average_x = average_x + lon_to_x(position_of_object);
            average_y = average_y + lat_to_y(position_of_object);
            flushinput();
            ++num;
        }
        draw_street_segment(p1, p2);
    }
}

void search_bar_search(std::string s) {
    opened_found_path = false;
   
    if (!do_closest_POI_search) {
        reset_highlights();
        saved_POI.clear();
        keep = true;

        std::vector<std::string> words;

        boost::split(words, s, boost::is_any_of("&"));
        if (words.size() == 1 && s.find("and") != std::string::npos) {
            words[0] = s.substr(0, s.find("and"));
            words.push_back(s.substr(s.find("and") + 3, s.size()));
        }
        if (words.size() != 1) { //intersection search
            std::string street1 = fix_string(words[0]);
            std::string street2 = fix_string(words[1]);
            found_search = find_intersection_ids_from_street_names(street1, street2);
            if (found_search.size() > 0) {
                shouldFind = true;
                search_method = INTERSECTION_SEARCH;
                update_message(street1 + " & " + street2 + " found");
            } else {
                update_message(street1 + " & " + street2 + " not found");
            }
            draw_screen();
            flushinput();
            
            return;
        } else { //look thru pois and street names for a match
            std::vector<unsigned> ids;
            bool search_done = false;
            s = fix_string(s);
            try {
                ids = streetID_from_streetName.at(s);
                //DO STUFF TO HIGHLIGHT STREET HERE
                search_method = STREETS_SEARCH;
                search_done = true;
                object_name_1 = fix_string(s);
                found_search = find_street_ids_from_name(object_name_1);
                if (found_search.size() > 0) shouldFind = true;
                if (shouldFind) update_message(object_name_1 + " found", 0);
                else update_message(object_name_1 + " not found", 0);
                draw_screen();
                
                return;
            } catch (std::out_of_range e) {
                if ((POI_list.find(s)) != POI_list.end()) {
                    ids = POI_list.at(s);
                }
                //DO STUFF TO HIGHLIGHT POIS HERE
                search_method = POI_SEARCH;
                object_name_1 = fix_string(s);
                for (unsigned index = 0; index < getNumberOfPointsOfInterest(); ++index) {
                    std::string POI_name = getPointOfInterestName(index);
                    POI_name = fix_string(POI_name);
                    if (POI_name == object_name_1) {
                        found_search.push_back(index);
                    }
                }
                if (found_search.size() > 0) shouldFind = true;
                if (shouldFind) update_message("Point of Interest found", 0);
                else update_message("Point of Interest not found", 0);
                draw_screen();
               
                return;
            }
            if (!search_done) {
                try {
                    ids = POI_list.at(s);
                    //DO STUFF TO HIGHLIGHT POIS HERE
                    search_method = POI_SEARCH;
                    object_name_1 = fix_string(s);
                    for (unsigned index = 0; index < getNumberOfPointsOfInterest(); ++index) {
                        std::string POI_name = getPointOfInterestName(index);
                        POI_name = fix_string(POI_name);
                        if (POI_name == object_name_1) {
                            found_search.push_back(index);
                        }
                    }
                    if (found_search.size() > 0) shouldFind = true;
                    if (shouldFind) update_message("Search found", 0);
                    else update_message("Search not found", 0);
                    draw_screen();
                    
                    return;
                } catch (std::out_of_range e) {
                    update_message("Search failed");
                }
            }
        }
    } else {
        if (do_closest_POI_search) {
            if (saved_intersections.size() == 1) {
                reset_highlights();
                saved_POI.clear();
                s = fix_string(s);
                try {
                    POI_list.at(s);
                    draw_closest_POI_to_intersection_path = true;
                    found_closest_POI_to_intersection_path = find_path_to_point_of_interest(saved_intersections[0],
                            s, TURN_PENALTY);
                    directions = all_turns(found_closest_POI_to_intersection_path);
                    selected_intersection = true;
                    opened_found_path = true;
                    max_segments = directions.size();
                    has_opened = false;
                    if (max_segments == 0) {
                        update_message("Either you have arrived at your destination, or there is no valid path", 0);
                    } else update_message(directions[0]);
                    draw_screen();
                    
                    return;
                } catch (std::out_of_range e) {
                    //POI doesnt exit
                    keep_message = true;
                    update_message(s + " is not a valid Point of Interest Name");
                }
            }
            do_closest_POI_search = false;
        } else {
            keep_message = true;
            update_message("Please have an intersection pinned first to do intersection to POI search");
        }
        
        return;
    }

   

}
