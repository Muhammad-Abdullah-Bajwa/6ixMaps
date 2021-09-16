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
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"
#include <sstream>
#include <chrono>
#define TURN_PENALTY 15
//pass in start time to make sure thread kicks out before taking too long

struct thread_argument {
    std::vector<unsigned> tour;
    std::vector<unsigned> tour2;
    bool first_iter;
    std::vector<DeliveryInfo> deliveries;
    double turn_penalty;
    std::vector<unsigned> result;
    unsigned start_loc;
};

std::unordered_map<unsigned, std::vector<unsigned>> pickups_dropoffs;
std::unordered_map<unsigned, std::vector<unsigned>> dropoffs_pickups;
std::map<unsigned, std::vector<std::pair<unsigned, std::vector<StreetSegmentIndex>>>> fastest_paths_list;
std::unordered_set<unsigned> pickups;
std::unordered_set<unsigned> dropoffs_list;

bool solution_sort(std::pair<std::vector<unsigned>, double> p1, std::pair<std::vector<unsigned>, double> p2);
double tour_cost_finder(std::vector<unsigned> path, double turn_penalty);
std::vector<unsigned> add_depots(std::vector<unsigned> tour, std::vector<unsigned> depots, double turn_penalty);
bool tour_legality(std::vector<unsigned> tour, const std::vector<DeliveryInfo>& deliveries);
unsigned find_target(std::vector<unsigned> path, unsigned start);
std::vector<unsigned> find_path_between_all_intersections(const unsigned intersect_id_start, std::unordered_set<unsigned> target_list, const double turn_penalty);
void fastest_list_init(const std::vector<DeliveryInfo>& deliveries, const float turn_penalty);
static void* thread1(void* package);
static void* thread2(void* package);
static void* thread3(void* package);
static void* thread4(void* package);
static void* thread5(void* package);
static void* thread6(void* package);
static void* thread7(void* package);

/**
 * 
 */

bool solution_sort(std::pair<std::vector<unsigned>, double> p1, std::pair<std::vector<unsigned>, double> p2) {
    if (p1.second < p2.second) {
        return true;
    } else {
        return false;
    }
}

std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries,
        const std::vector<unsigned>& depots,
        const float turn_penalty) {
    std::vector<unsigned> empty;
    //std::cout << deliveries.size() << "\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    fastest_list_init(deliveries, turn_penalty);
    auto time2 = std::chrono::high_resolution_clock::now();
    
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - start_time).count() << "\n";

    //Need to figure out closest depot then from there
    //search for the closest pick up (remember to update sets for drop off)
    //Now check if the drop off or next pick up is closer
    //And go to that, then search for the next closest pick up

    std::vector<unsigned> path; //This stores all the paths you will go to
    std::vector<unsigned> valid;

    for (unsigned i = 0; i < deliveries.size(); ++i) {
        pickups_dropoffs[deliveries[i].pickUp].push_back(deliveries[i].dropOff);
        dropoffs_pickups[deliveries[i].dropOff].push_back(deliveries[i].pickUp);
        //std::cout << deliveries[i].dropOff << " " << deliveries[i].pickUp<<"\n";
        if (pickups.find(deliveries[i].pickUp) == pickups.end()) {
            pickups.insert(deliveries[i].pickUp);
        }
        dropoffs_list.insert(deliveries[i].dropOff);
        valid.push_back(deliveries[i].pickUp);
    }
    std::vector<unsigned> starting_path;
    unsigned count = 0;
    do {
        std::vector<unsigned> targets(pickups.begin(), pickups.end());
        starting_path = find_path_between_intersections(depots[count], targets, turn_penalty); //FIX THIS TO SELECT BEST DEPOT
        count++;
        if (count > depots.size()) return empty;
    } while (starting_path.empty());

    path.push_back(depots[count - 1]);

    std::unordered_set<unsigned> valid_pickups = pickups;
    std::unordered_set<unsigned> valid_dropoffs;
    int counter = 0;
    while (!valid.empty()) {
        //std::cout<< "\n";
        valid.clear();
        valid.insert(valid.end(), valid_pickups.begin(), valid_pickups.end());
        valid.insert(valid.end(), valid_dropoffs.begin(), valid_dropoffs.end());


        if (valid.empty()) {
            break;
        }
        /*std::cout << "VALID: ";
        for (unsigned i = 0; i < valid.size(); i++) {
            std::cout << valid[i] << " ";
        }
        std::cout << "\n";
        std::cout << "PATH: ";
        for (unsigned i = 0; i < path.size(); i++) {
            std::cout << path[i] << " ";
        }
        std::cout << "\n";*/

        std::vector<unsigned> closest_pick_up_from_depot = find_path_between_intersections(path.back(), valid, turn_penalty);
        counter++;
        //std::cout << counter << "\n";
        unsigned next_stop = find_target(closest_pick_up_from_depot, path.back());

        if (valid_pickups.find(next_stop) != valid_pickups.end()) {
            valid_pickups.erase(next_stop);
            std::vector<unsigned> add_to_dropoffs = pickups_dropoffs[next_stop];
            for (unsigned i = 0; i < add_to_dropoffs.size(); i++) {
                valid_dropoffs.insert(add_to_dropoffs[i]);
            }
            valid_pickups.erase(next_stop);
        }
        if (valid_dropoffs.find(next_stop) != valid_dropoffs.end()) {

            std::vector<unsigned> remove_pickups = dropoffs_pickups[next_stop];
            for (unsigned i = 0; i < remove_pickups.size(); i++) {
                if (valid_pickups.find(remove_pickups[i]) == valid_pickups.end()) {
                    valid_pickups.erase(remove_pickups[i]);
                }
            }
            valid_dropoffs.erase(next_stop);
        }
        path.push_back(next_stop);
    }


    /*std::vector<unsigned> final_path;*/
    path.erase(path.begin(), path.begin() + 1);
    /* std::cout << "greedy path: ";
     for (unsigned i = 0; i < path.size(); ++i) {
         // std::vector<unsigned>temp = find_path_between_intersections(path[i], path[i + 1], turn_penalty);
         //final_path.insert(final_path.end(), temp.begin(), temp.end());
         std::cout << path[i] << " ";
     }
     std::cout << "\n";*/
    //return final_path;
    auto time3 = std::chrono::high_resolution_clock::now();
    //std::cout << "IMPROVEMENT TIME! " << std::chrono::duration_cast<std::chrono::milliseconds>(time3 - start_time).count() << "\n";

    //solution improvement
    std::vector<unsigned> best_known_path = path;
    double best_cost = tour_cost_finder(path, turn_penalty);
    std::vector<unsigned> localSearch_path = path;
    std::vector<std::pair<std::vector<unsigned>, double> > sol_list;
    count = 0;
    int kick = 0;
    int perturbation = 0;
    int improvement = 0;
    while (1 > 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() > 29000) {
            //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() << " asdfsdf\n";
            //std::cout << "PERTURBATIONS: " << perturbation << "\n";
            //std::cout << improvement << " improvements\n";
            break; //from improvement loop
        }

        //std::cout<<"thread start\n";
        auto after_time = std::chrono::high_resolution_clock::now();

        pthread_t threads[8];
        pthread_attr_t attr;
        void *status;

        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

        thread_argument* arg1 = new thread_argument;
        arg1->deliveries.insert(arg1->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg1->tour = localSearch_path;
        arg1->turn_penalty = turn_penalty;

        pthread_create(&threads[0], NULL, thread1, static_cast<void*> (arg1));

        thread_argument* arg2 = new thread_argument;
        arg2->deliveries.insert(arg2->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg2->tour = localSearch_path;
        arg2->turn_penalty = turn_penalty;

        pthread_create(&threads[1], NULL, thread7, static_cast<void*> (arg2));

        thread_argument* arg3 = new thread_argument;
        arg3->deliveries.insert(arg3->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg3->tour = localSearch_path;
        arg3->turn_penalty = turn_penalty;

        pthread_create(&threads[2], NULL, thread1, static_cast<void*> (arg3));

        thread_argument* arg4 = new thread_argument;
        arg4->deliveries.insert(arg4->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg4->tour = localSearch_path;
        arg4->turn_penalty = turn_penalty;

        pthread_create(&threads[3], NULL, thread7, static_cast<void*> (arg4));

        thread_argument* arg5 = new thread_argument;
        arg5->deliveries.insert(arg5->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg5->tour = localSearch_path;
        arg5->turn_penalty = turn_penalty;

        pthread_create(&threads[4], NULL, thread1, static_cast<void*> (arg5));

        thread_argument* arg6 = new thread_argument;
        arg6->deliveries.insert(arg6->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg6->tour = localSearch_path;
        arg6->turn_penalty = turn_penalty;

        pthread_create(&threads[5], NULL, thread7, static_cast<void*> (arg6));


        thread_argument* arg7 = new thread_argument;
        arg7->deliveries.insert(arg7->deliveries.begin(), deliveries.begin(), deliveries.end());
        arg7->tour = localSearch_path;
        arg7->turn_penalty = turn_penalty;


        pthread_create(&threads[6], NULL, thread1, static_cast<void*> (arg7));


        pthread_attr_destroy(&attr);
        sol_list.clear();


        pthread_join(threads[0], &status);
        std::vector<unsigned> sol1 = arg1->result;
        double new_cost = tour_cost_finder(sol1, turn_penalty);
        sol_list.push_back(std::make_pair(sol1, new_cost));

        pthread_join(threads[1], &status);
        std::vector<unsigned> sol2 = arg2->result;
        double new_cost1 = tour_cost_finder(sol2, turn_penalty);
        sol_list.push_back(std::make_pair(sol2, new_cost1));

        pthread_join(threads[2], &status);
        std::vector<unsigned> sol3 = arg3->result;
        double new_cost2 = tour_cost_finder(sol3, turn_penalty);
        sol_list.push_back(std::make_pair(sol3, new_cost2));

        pthread_join(threads[3], &status);
        std::vector<unsigned> sol4 = arg4->result;
        double new_cost3 = tour_cost_finder(sol4, turn_penalty);
        sol_list.push_back(std::make_pair(sol4, new_cost3));

        pthread_join(threads[4], &status);
        std::vector<unsigned> sol5 = arg5->result;
        double new_cost4 = tour_cost_finder(sol5, turn_penalty);
        sol_list.push_back(std::make_pair(sol5, new_cost4));

        pthread_join(threads[5], &status);
        std::vector<unsigned> sol6 = arg6->result;
        double new_cost5 = tour_cost_finder(sol6, turn_penalty);
        sol_list.push_back(std::make_pair(sol6, new_cost5));

        pthread_join(threads[6], &status);
        std::vector<unsigned> sol7 = arg7->result;
        double new_cost6 = tour_cost_finder(sol7, turn_penalty);
        sol_list.push_back(std::make_pair(sol7, new_cost6));


        delete arg1;
        delete arg2;
        delete arg3;
        delete arg4;
        delete arg5;
        delete arg6;
        delete arg7;

        std::sort(sol_list.begin(), sol_list.end(), solution_sort);

        if (sol_list[0].second == best_cost) {
            count++;
            kick++;
        }

        if (sol_list[0].second < best_cost) {
            //std::cout << "NEW PATH FOUND!\n";
            improvement++;
            best_known_path = sol_list[0].first;
            best_cost = sol_list[0].second;
            count = 0;
            kick = 0;
        }
        
        if (kick >= 1000){
            break;
        }
        
        
        auto time2x = std::chrono::high_resolution_clock::now();
        double time_rem = 30000 - std::chrono::duration_cast<std::chrono::milliseconds>(time2x - start_time).count();
        double probability = exp((best_cost - sol_list[0].second) / (time_rem));
        //std::cout << probability << " BEST COST: " << best_cost << " NEW COST: " << sol_list[0].second << " TIME REMAINING: " << time_rem << "\n";
        std::random_device rd;
        std::mt19937 re(rd());
        std::uniform_real_distribution<double> unif(0, 1);
        double num = unif(re);
        //std::cout << "NUM: " << num << "\n";
        if (sol_list[0].second < best_cost) {
            localSearch_path = localSearch_path = sol_list[0].first;
        } else if (count >= 10) {
            localSearch_path = sol_list[1].first;
            count = 0;
        } else if (num < probability) {
            localSearch_path = sol_list[0].first;
            //std::cout << "\nEXPLORING!\n\n";
        } else {

            localSearch_path = localSearch_path;
        }


        /*
     
         multi threaded calls to change functions here
     
     
         */


        //make sure to check each solution with best known solution

        //keep track how many times the best solution of 8 matches the best solution known to date, its possible we are at the global minimum

        /*std::vector<unsigned> weights = {20, 17}; //, 13, 11, 11, 11, 9, 8};
        int weight_sum = 0;
        current_time = std::chrono::high_resolution_clock::now();
        double time_left = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        for (unsigned i = 0; i < weights.size(); i++) {
            weights[i] = weights[i] + round((time_left / 1000) * i / 24);
            weight_sum = weight_sum + weights[i];
        }

        std::uniform_int_distribution<std::mt19937::result_type> distribution(0, weight_sum - 1);
        std::mt19937 rng;
        rng.seed(std::random_device()());
        unsigned rand = distribution(rng);
        unsigned index = 0;
        for (unsigned i = 0; i < weights.size(); i++) {
            if (rand < weights[i]) {
                index = i;
            }
            rand = rand - weights[i];
        }*/
        //std::cout << "NEW SOLUTION TO EXPLORE! " << index << "\n";
        //localSearch_path = sol_list[0].first;
        perturbation++;

    }

    if (best_known_path.empty()) {
        return best_known_path;
    }

    /*for (int i = 0; i < best_known_path.size(); i++) {
        std::cout << best_known_path[i] << "\n";
    }*/


    std::vector<unsigned> full_tour = add_depots(best_known_path, depots, turn_penalty);
    std::vector<unsigned> result;

    result = find_path_between_intersections(full_tour[0], full_tour[1], turn_penalty);
    for (unsigned i = 1; i < full_tour.size() - 2; i++) {

        std::vector<unsigned> temp;
        std::vector<std::pair<unsigned, std::vector<unsigned>>> next_stops = fastest_paths_list[full_tour[i]];
        for (unsigned j = 0; j < next_stops.size(); j++) {
            //std::cout << "ADJACENT: " << next_stops[j].first << "\n";
            if (next_stops[j].first == full_tour[i + 1]) {
                temp = next_stops[j].second;
                break;
            }
        }
        result.insert(result.end(), temp.begin(), temp.end());
    }
    std::vector<unsigned> temp = find_path_between_intersections(full_tour[full_tour.size() - 2], full_tour.back(), turn_penalty);
    result.insert(result.end(), temp.begin(), temp.end());
    //std::cout << getStreetSegmentInfo(result[0]).from << " " << getStreetSegmentInfo(result[0]).to <<" " <<  getStreetSegmentInfo(result[1]).from << " " << getStreetSegmentInfo(result[1]).to;

    /*std::cout << "best_tour: ";
    for (unsigned k = 0; k < best_known_path.size(); k++) {
        std::cout << best_known_path[k] << " ";
    }
    std::cout << "\n\n";*/
    valid_dropoffs.clear();
    pickups.clear();
    pickups_dropoffs.clear();
    dropoffs_pickups.clear();
    fastest_paths_list.clear();
    return result;
}

unsigned find_target(std::vector<unsigned> path, unsigned start) {
    if (path.size() == 1) {
        StreetSegmentInfo s = getStreetSegmentInfo(path[0]);
        if (s.from == start) {
            return s.to;
        } else {
            return s.from;
        }
    } else {
        //std::cout << path.size() << " PATH SIZE IN FIND TARGET FUNC\n";
        StreetSegmentInfo last_seg = getStreetSegmentInfo(path.back());
        StreetSegmentInfo other_seg = getStreetSegmentInfo(path[path.size() - 2]);
        //std::cout << last_seg.to << " " << last_seg.from << " " << other_seg.to << " " << other_seg.from << "\n";
        if (last_seg.from == other_seg.from) {
            return last_seg.to;
        } else if (last_seg.from == other_seg.to) {
            return last_seg.to;
        } else if (last_seg.to == other_seg.to) {
            return last_seg.from;
        } else {
            return last_seg.from;
        }
    }
}


//package is going to be all of the arguments packed into one variable
// struct package { tour, deliveries}
//returns new tour

static void* thread1(void* package) {

    thread_argument* arg = static_cast<thread_argument*> (package);
//    auto start_time = std::chrono::high_resolution_clock::now();


    int start_index;
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(0, arg->tour.size() - 2);

    start_index = dist6(rng);
    std::vector<unsigned> new_tour = arg->tour;
    //std::cout << "SELECTED INTERSECTion: " << arg->tour[start_index]<<"\n";

    if (pickups.find(arg->tour[start_index]) != pickups.end() && dropoffs_list.find(arg->tour[start_index]) != dropoffs_list.end()) {

    } else if (pickups.find(arg->tour[start_index]) != pickups.end()) {

        if (dropoffs_list.find(arg->tour[start_index + 1]) != dropoffs_list.end()) {
            //std::cout << start_index + 1 << "\n";
            std::vector<unsigned> check = dropoffs_pickups[arg->tour[start_index + 1]];
            //std::cout << check.size() << "\n";
            for (unsigned i = 0; i < check.size(); i++) {
                //std::cout<<check[i]<<"\n";
                if (check[i] == arg->tour[start_index]) {
                    arg->result = arg->tour;
                    return (static_cast<void*> (arg->tour.data()));
                }
            }

        }
        std::reverse(new_tour.begin() + start_index, new_tour.begin() + start_index + 2);
    } else {

        if (pickups.find(arg->tour[start_index + 1]) != pickups.end()) {
            std::vector<unsigned> check = pickups_dropoffs[arg->tour[start_index + 1]];
            for (unsigned i = 0; i < check.size(); i++) {
                if (check[i] == arg->tour[start_index]) {
                    arg->result = arg->tour;
                    return (static_cast<void*> (arg->tour.data()));
                }
            }

        }
        std::reverse(new_tour.begin() + start_index, new_tour.begin() + start_index + 2);
    }
    arg->result = new_tour;
    return (static_cast<void*> (new_tour.data()));


}

static void* thread2(void* package) {
    thread_argument* arg = static_cast<thread_argument*> (package);
    std::vector<unsigned> result;

    if (arg->first_iter) { //DO RANDOM SWAP

        arg->result = arg->tour;
        //std::cout << "first exit \n";
        return static_cast<void*> (arg->tour.data());
    } else {
        //std::cout <<"THREAD2 BEGUN\n";
        int rand_bound1 = round(0.3 * arg->tour.size());
        int rand_bound2 = round(0.6 * arg->tour.size());
        std::mt19937 rng;
        rng.seed(std::random_device()());
        std::uniform_int_distribution<std::mt19937::result_type> dist6(rand_bound1, rand_bound2);

        int cpy_index = dist6(rng);
        result.insert(result.end(), arg->tour.begin(), arg->tour.begin() + cpy_index);
        std::unordered_set<unsigned> temp(result.begin(), result.end());
        std::cout << arg->tour2.size() << "\n";
        for (unsigned i = 0; i < arg->tour2.size(); i++) {
            if (temp.find(arg->tour2[i]) == temp.end()) {
                temp.insert(arg->tour2[i]);
                result.push_back(arg->tour2[i]);
            }
        }
        std::cout << "NEW TOUR thread2: ";
        for (unsigned i = 0; i < result.size(); i++) {
            std::cout << result[i] << " ";
        }
        std::cout << "\n";
        arg->result = result;
        return static_cast<void*> (result.data());
    }
}


//computes path cost from a vector of intersection ids

double tour_cost_finder(std::vector<unsigned> path, double turn_penalty) {
    double cost = 0;
    //std::cout<<"COST FINDER PATH SIZE: " << path.size() << "\n";
    for (unsigned i = 0; i < path.size() - 1; i++) {
        //std::cout << path[i] << " ";
        std::vector<unsigned> segments_path;
        std::vector<std::pair<unsigned, std::vector<unsigned>>> next_stops = fastest_paths_list[path[i]];
        for (unsigned j = 0; j < next_stops.size(); j++) {
            //std::cout << "ADJACENT: " << next_stops[j].first << "\n";
            if (next_stops[j].first == path[i + 1]) {
                segments_path = next_stops[j].second;
                break;
            }
        }


        cost = cost + compute_path_travel_time(segments_path, turn_penalty);
    }
    //std::cout << cost <<"\n";
    return cost;
}

//tour legality function (ASSUMES NO DEPOTS AT END)

bool tour_legality(std::vector<unsigned> tour, const std::vector<DeliveryInfo>& deliveries) {
    std::unordered_set<unsigned> pickups1; //these should be global
    std::unordered_map<unsigned, unsigned> dropoffs;
    for (unsigned i = 0; i < deliveries.size(); i++) {
        pickups1.insert(deliveries[i].pickUp);
        dropoffs[deliveries[i].dropOff] = deliveries[i].pickUp;
    }
    std::unordered_set<unsigned> done_pickups;
    for (unsigned i = 0; i < tour.size(); i++) {
        std::unordered_set<unsigned>::iterator it = pickups1.find(tour[i]);
        if (it == pickups1.end()) {
            unsigned pickup = dropoffs[tour[i]]; //ASSUMING TOUR HAS INTERSECTIONS THAT ARE PART OF DELIVERIES AND NO OTHER INTERSECTIONS
            if (done_pickups.find(pickup) == done_pickups.end()) { //if drop is made before pickup
                return false;
            }
        } else {
            done_pickups.insert(*it);
        }
    }
    return true;
}

//adds depots to the two ends of the tour

std::vector<unsigned> add_depots(std::vector<unsigned> tour, std::vector<unsigned> depots, double turn_penalty) {
    double cost = std::numeric_limits<double>::infinity();
    unsigned start_depot = 0;
    std::vector<unsigned> segments_path;
    for (unsigned i = 0; i < depots.size(); i++) {
        std::vector<unsigned> temp = find_path_between_intersections(depots[i], tour[0], turn_penalty);
        if (!temp.empty()) {
            double new_cost = compute_path_travel_time(temp, turn_penalty);
            if (new_cost < cost) {
                start_depot = i;
                segments_path = temp;
                cost = new_cost;
            }
        }
    }

    unsigned depot_init = depots[start_depot]; //function call
    segments_path = find_path_between_intersections(tour.back(), depots, turn_penalty);
    unsigned depot_final = find_target(segments_path, tour.back()); //function call
    //std::cout<<depot_final<<"\n";
    tour.push_back(depot_final);
    std::vector<unsigned> result;
    result.push_back(depot_init);
    result.insert(result.begin() + 1, tour.begin(), tour.end());
    return result;
}

std::vector<unsigned> find_path_between_all_intersections(const unsigned intersect_id_start,
        std::unordered_set<unsigned> target_list,
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
    //std::cout<<"TARGETS SIZE " << target_list.size() << "\n";

    heap.push(std::make_pair(0, intersect_id_start));
    dijkstraValue[intersect_id_start] = 0;
    IntersectionIndex currentNode;
    while (!heap.empty()) {

        currentNode = heap.top().second;
        heap.pop();

        //std::cout<<"CURRENT NODE " << currentNode << "\n";


        std::unordered_set<unsigned>::iterator element = target_list.find(currentNode);

        if (element != target_list.end()) {

            std::vector<unsigned> temp;
            IntersectionIndex index = currentNode;
            std::deque<Node_Edge_Object> node_list = adjacency_list[index];
            while (index != intersect_id_start) {
                temp.push_back(parents[index].first);
                index = parents[index].second;
            }
            std::reverse(temp.begin(), temp.end());
            fastest_paths_list[intersect_id_start].push_back(std::make_pair(*element, temp));
            target_list.erase(element);

            //std::cout<<"ADDED PATH SIZE: " << temp.size() <<" FROM " << intersect_id_start << " TO " << *element<<  "\n";
        }
        if (target_list.empty()) {
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
    return path;
}

void fastest_list_init(const std::vector<DeliveryInfo>& deliveries, const float turn_penalty) {
    std::unordered_set<unsigned> unique_intersections;
    std::vector<unsigned> unique_intersections_vector;
    //std::cout<<"CALLED INIT FUNC\n";
    for (unsigned i = 0; i < deliveries.size(); i++) {
        if (unique_intersections.find(deliveries[i].pickUp) == unique_intersections.end()) {
            unique_intersections.insert(deliveries[i].pickUp);
            unique_intersections_vector.push_back(deliveries[i].pickUp);
        }
        if (unique_intersections.find(deliveries[i].dropOff) == unique_intersections.end()) {
            unique_intersections.insert(deliveries[i].dropOff);
            unique_intersections_vector.push_back(deliveries[i].dropOff);
        }
    }
    //std::cout << "NUMBER OF CALLS: " << unique_intersections_vector.size() << "\n";
#pragma omp parallel for
    for (unsigned i = 0; i < unique_intersections_vector.size(); i++) {
        std::unordered_set<unsigned> temp = unique_intersections;
        temp.erase(unique_intersections_vector[i]);
        //auto start_time = std::chrono::high_resolution_clock::now();
        find_path_between_all_intersections(unique_intersections_vector[i], temp, turn_penalty);
        /*auto time2 = std::chrono::high_resolution_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - start_time).count() << " FIND TIME\n";*/
    }

}
//package is going to be all of the arguments packed into one variable
// struct package { tour, deliveries}
//returns new tour
//implements insertion opt

static void* thread3(void* package) {
    thread_argument* arg = static_cast<thread_argument*> (package);
    std::unordered_set<unsigned> pickup_list;
    std::unordered_map<unsigned, std::vector<unsigned>> drop_pickup;
    std::unordered_map<unsigned, unsigned> done_pickup_loc;
    auto start_time = std::chrono::high_resolution_clock::now();

    //std::cout << "PICKUPS: ";
    for (unsigned i = 0; i < arg->deliveries.size(); i++) {
        unsigned pick = arg->deliveries[i].pickUp;
        unsigned drop = arg->deliveries[i].dropOff;
        pickup_list.insert(pick);
        //std::cout << pick << " ";
        drop_pickup[drop].push_back(pick);
    }
    //std::cout<<"\n";
    std::vector<unsigned> best_tour = (*arg).tour;

    std::vector<unsigned> side_list;
    std::vector<unsigned> new_list;

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(0, best_tour.size() - 3);
    bool legal_pathing = false;
    //finding legal path
    unsigned counter1 = 0;
    while (legal_pathing == false && counter1 < (new_list.size() - 1)) {
        auto current_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() > 2500) {
            arg->result = best_tour;
            return (static_cast<void*> (best_tour.data()));
        }

        side_list.clear();
        new_list.clear();
        new_list = best_tour;
        counter1++;
        unsigned rand = distribution(rng);

        int counter = 0;
        while (counter < 3 && (counter + rand) < new_list.size()) {
            counter++;
        }
        side_list.insert(side_list.begin(), new_list.begin() + rand, new_list.begin() + rand + counter);
        new_list.erase(new_list.begin() + rand, new_list.begin() + rand + counter);
        //insert the side list back into new_list then do the legal check
        std::uniform_int_distribution<std::mt19937::result_type> dist(0, new_list.size() - 1);
        rand = dist(rng);
        new_list.insert(new_list.begin() + rand, side_list.begin(), side_list.end());

        std::map<unsigned, unsigned> locations;
        for (unsigned i = 0; i < new_list.size(); ++i) {
            locations[new_list[i]] = i;
        }
        int numdrop = 0;
        int numeach = 0;
        //pickup_dropoffs
        //drop_pickup
//        auto current_time1 = std::chrono::high_resolution_clock::now();
        for (unsigned i = 0; i < side_list.size(); ++i) {
            //if drop off
            if (drop_pickup.find(side_list[i]) != drop_pickup.end()) {
                numdrop++;
                //if the drop of index is greater than any of its pick ups
                for (unsigned j = 0; j < drop_pickup[side_list[i]].size(); ++j) {
                    //finding pick up with largest index
                    unsigned p = 0;
                    for (unsigned k = 1; (k) < locations.size(); ++k) {
                        unsigned previous = locations[drop_pickup[side_list[i]][0]];
                        if (previous < locations[drop_pickup[side_list[i]][k]]) {
                            p = k;
                        }
                    }
                    //Comparing to see if dropoff is after the largest indexed pickup
                    if (locations[side_list[i]] > locations[drop_pickup[side_list[i]][p]]) {
                        numeach++;
                        break;
                    }
                }
            }
        }
        if (numeach == numdrop) legal_pathing = true;
        //
        //check for if the side_list has any dropoffs
    }

    if (!legal_pathing) return (static_cast<void*> (best_tour.data()));
    //std::cout << "counter: " << counter1 << " rand: " << rand << '\n';
    /*std::cout << "new_tour: ";
    for (unsigned k = 0; k < new_list.size(); k++) {
        std::cout << new_list[k] << " ";
    }
    std::cout << "\n\n";*/
    double new_cost = tour_cost_finder(new_list, (*arg).turn_penalty);
    best_tour = new_list;
    /*    std::cout << "new_tour: ";
        for (unsigned k = 0; k < best_tour.size(); k++) {
            std::cout << best_tour[k] << " ";
        }
        std::cout << "\n\n";
     */
    arg->result = best_tour;
    auto current_time2 = std::chrono::high_resolution_clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(current_time2 - current_time1).count() << '\n';
    return (static_cast<void*> (best_tour.data()));
}

static void* thread4(void* package) {
    thread_argument* arg = static_cast<thread_argument*> (package);

    std::vector<unsigned> best_tour = (*arg).tour;
    std::vector<unsigned>::iterator it;
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(0, best_tour.size() - 1);
    bool pick = false;
    while (pick == false) {
        unsigned rand = distribution(rng);
        std::vector<unsigned>::iterator hold = best_tour.begin() + rand;
        unsigned hold_pickup = *(hold);
        //if node is a pick up
        if (pickups_dropoffs.find(hold_pickup) != pickups_dropoffs.end()) {
            //erase it from the list
            best_tour.erase(hold);
            //add it to the front
            best_tour.insert(best_tour.begin(), hold_pickup);
            //find corresponding drop offs
            std::vector<unsigned>::iterator hold_drop = std::find(best_tour.begin() + 1, best_tour.end(), pickups_dropoffs[hold_pickup][0]);
            if (hold_drop != best_tour.end()) {
                //erase it from current location
                best_tour.erase(hold_drop);
                //add it to back
                best_tour.push_back(*hold_drop);
                pick = true;
            }
        } else pick = false;
    }
    double new_cost = tour_cost_finder(best_tour, (*arg).turn_penalty);

    /*    std::cout << "new_tour: ";
        for (unsigned k = 0; k < best_tour.size(); k++) {
            std::cout << best_tour[k] << " ";
        }
        std::cout << "\n\n";
     */
    arg->result = best_tour;
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
    return (static_cast<void*> (best_tour.data()));
}

static void* thread5(void* package) {
    //auto start_time = std::chrono::high_resolution_clock::now();
    //completely random
    thread_argument* arg = static_cast<thread_argument*> (package);
    std::vector<unsigned> result;
    std::vector<unsigned> valid;
    std::unordered_set<unsigned> temp_pickups = pickups;
    std::unordered_set<unsigned> temp_dropoffs;


    valid.insert(valid.begin(), pickups.begin(), pickups.end());
    //std::cout<<arg->deliveries.size()<<"\n";
    while (!valid.empty()) {
        //std::cout<<"adsadsa\n";
        std::unordered_set<unsigned> valid_locs;
        valid_locs.insert(temp_pickups.begin(), temp_pickups.end());
        valid_locs.insert(temp_dropoffs.begin(), temp_dropoffs.end());

        std::mt19937 rng;
        rng.seed(std::random_device()());
        std::uniform_int_distribution<std::mt19937::result_type> dist6(0, valid_locs.size() - 1);
        unsigned index = dist6(rng);
        //std::cout<<"INDEX: " << index << " VALID SIZE " << valid.size()<< "\n";
        unsigned inters = valid[index];
        //std::cout << inters << "\n";
        result.push_back(inters);

        if (temp_pickups.find(inters) != temp_pickups.end()) {
            std::vector<unsigned> allDropoffs = pickups_dropoffs[inters];
            //std::cout<<"ALL DROPS FOR " << inters <<": ";
            for (unsigned i = 0; i < allDropoffs.size(); i++) {
                temp_dropoffs.insert(allDropoffs[i]);
                //std::cout<<allDropoffs[i]<<"\n";
            }
            //std::cout << temp_pickups.size() << " BEFORE\n";
            temp_pickups.erase(inters);
            //std::cout << temp_pickups.size() << " AFTER\n";
        }
        if (temp_dropoffs.find(inters) != temp_dropoffs.end()) {
            temp_dropoffs.erase(inters);
        }

        valid.clear();
        valid.insert(valid.end(), temp_pickups.begin(), temp_pickups.end());
        valid.insert(valid.end(), temp_dropoffs.begin(), temp_dropoffs.end());
    }
    /*std::cout<<"RESULT: ";
    for (int i=0; i<result.size();i++){
        std::cout<<result[i]<<" ";
    }
    std::cout<<"\n";*/
    arg->result = result;
    //auto time2 = std::chrono::high_resolution_clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - start_time).count() << " THREAD TIME\n";
    return static_cast<void*> (result.data());
}

static void* thread6(void* package) {
    thread_argument* arg = static_cast<thread_argument*> (package);
    std::unordered_map<unsigned, unsigned> inter_to_index;
    std::vector<unsigned> result;
    std::vector<unsigned>::iterator index;
    double cost = 0;
    //std::cout << arg->tour.size() << "\n";
    for (std::vector<unsigned>::iterator i = arg->tour.begin(); i != arg->tour.end() - 1; i++) {
        //std::cout <<"daagasgdas" << "\n";
        std::vector<unsigned> temp;
        temp.push_back(*i);
        temp.push_back(*(i + 1));
        double new_cost = tour_cost_finder(temp, arg->turn_penalty);
        if (new_cost > cost) {
            cost = new_cost;
            index = i;

        }
        inter_to_index[*i] = i - arg->tour.begin();
    }
    std::cout << "INDEX: " << *index << "\n";
    unsigned inters = *index;
    //std::cout << "INTERS: " << inters << "\n";
    /*std::cout << "before       ";
    for (int i = 0; i < arg->tour.size(); i++) {
        std::cout << arg->tour[i] << " ";
    }
    std::cout << "\n";*/
    if (pickups.find(inters) != pickups.end() && dropoffs_list.find(inters) != dropoffs_list.end()) {
        std::vector<unsigned> last_drops_pickups = dropoffs_pickups[arg->tour.back()];
        unsigned last_pick = 0;
        for (unsigned i = 0; i < last_drops_pickups.size(); i++) {
            if (last_pick < inter_to_index[last_drops_pickups[i]]) {
                last_pick = inter_to_index[last_drops_pickups[i]];
            }
        }
        result.insert(result.end(), arg->tour.begin(), arg->tour.begin() + last_pick + 1);
        result.push_back(arg->tour.back());
        arg->tour.pop_back();
        result.insert(result.end(), arg->tour.begin() + last_pick + 1, arg->tour.end());
        std::cout << "after1       ";
        for (unsigned i = 0; i < result.size(); i++) {
            std::cout << result[i] << " ";
        }
        std::cout << "\n";
    } else if (pickups.find(inters) != pickups.end()) {
        std::vector<unsigned> all_drops = pickups_dropoffs[inters];
        unsigned first_drop = arg->tour.size();
        for (unsigned i = 0; i < all_drops.size(); i++) {
            if (first_drop > inter_to_index[all_drops[i]]) {
                first_drop = inter_to_index[all_drops[i]];
            }
        }
        std::vector< std::pair<unsigned, std::vector<StreetSegmentIndex> > >::iterator it;
        int new_pos = 0;
        for (it = fastest_paths_list[inters].begin(); it != fastest_paths_list[inters].end(); it++) {
            if (inter_to_index[(*it).first] < first_drop) {
                new_pos = inter_to_index[(*it).first] - 1;
                break;
            }
        }
        if (new_pos != -1) {
            result.insert(result.end(), arg->tour.begin(), arg->tour.begin() + new_pos);
            result.push_back(inters);
            arg->tour.erase(index);
            result.insert(result.end(), arg->tour.begin() + new_pos, arg->tour.end());
            std::cout << "after2       ";
            for (unsigned i = 0; i < result.size(); i++) {
                std::cout << result[i] << " ";
            }
            std::cout << "\n";
        } else if (new_pos == -1) {
            result.push_back(inters);
            arg->tour.erase(index);
            result.insert(result.end(), arg->tour.begin(), arg->tour.end());
            std::cout << "after3       ";
            for (unsigned i = 0; i < result.size(); i++) {
                std::cout << result[i] << " ";
            }
            std::cout << "\n";
        }
    } else {
        std::vector<unsigned> all_picks = dropoffs_pickups[inters];
        unsigned latest_pick = 0;
        for (unsigned i = 0; i < all_picks.size(); i++) {
            if (latest_pick < inter_to_index[all_picks[i]]) {
                latest_pick = inter_to_index[all_picks[i]];
            }
        }
        std::vector< std::pair<unsigned, std::vector<StreetSegmentIndex> > >::iterator it;
        int new_pos = 0;
        for (it = fastest_paths_list[inters].begin(); it != fastest_paths_list[inters].end(); it++) {
            if (inter_to_index[(*it).first] > latest_pick) {
                new_pos = inter_to_index[(*it).first] - 1;
                break;
            }
        }
        std::cout << "new pos" << new_pos << "\n";
        if ((new_pos) != -1) {
            result.insert(result.end(), arg->tour.begin(), arg->tour.begin() + new_pos);
            result.push_back(inters);
            arg->tour.erase(index);
            result.insert(result.end(), arg->tour.begin() + new_pos, arg->tour.end());
            std::cout << "after4       ";
            for (unsigned i = 0; i < result.size(); i++) {
                std::cout << result[i] << " ";
            }
            std::cout << "\n";
        } else if (new_pos == -1) {
            result.push_back(inters);
            arg->tour.erase(index);
            result.insert(result.end(), arg->tour.begin(), arg->tour.end());
            std::cout << "after5       ";
            for (unsigned i = 0; i < result.size(); i++) {
                std::cout << result[i] << " ";
            }
            std::cout << "\n";
        }
    }
    std::cout << "final        ";
    for (unsigned i = 0; i < result.size(); i++) {
        std::cout << result[i] << " ";
    }
    std::cout << "\n\n";
    arg->result = result;
    return static_cast<void*> (result.data());
}

//Reverse 3
static void* thread7(void* package) {
    thread_argument* args = static_cast<thread_argument*> (package);
    std::vector<unsigned> best_tour = args->tour;

    int start_index = 0;
    std::mt19937 rq;
    rq.seed(std::random_device()());
    if ((*args).tour.size() >= 4) {
        std::uniform_int_distribution<std::mt19937::result_type> dist6(0, (*args).tour.size() - 4);
        start_index = dist6(rq);
    } else if ((*args).tour.size() <= 3) {
        std::uniform_int_distribution<std::mt19937::result_type> dist6(0, (*args).tour.size() - 1);
        start_index = dist6(rq);
    }

    std::vector<unsigned>::iterator start_pos = best_tour.begin() + start_index;
    unsigned count = 0;
    while (start_pos + count != best_tour.end() && count < 3) {
        count++;
    }
    while (count > 0) {
        //check if the switched first one has pick ups at the two before it before reverse, if so do random generator again
        std::unordered_map<unsigned, std::vector<unsigned>>::iterator it = dropoffs_pickups.find(*(start_pos + count));
        if (it != dropoffs_pickups.end()) {
            //it is a drop off and now need to check for seeing if the one before it is a pick up
            for (unsigned i = 0; i < dropoffs_pickups[*(start_pos + count)].size(); ++i) {
                //if the value in the first position is in the list of the others pick ups rip
                if (*(start_pos + count) == dropoffs_pickups[*(start_pos + count)][i]) {
                    args->result = best_tour;
                    return (static_cast<void*> (best_tour.data()));
                }
            }
        }
        count--;
    }
    //check if the middle has pickups at the one before it, pre-reverse
    std::reverse(start_pos, start_pos + count);
    args->result = best_tour;
    return (static_cast<void*> (best_tour.data()));
}