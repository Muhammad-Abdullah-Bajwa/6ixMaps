#include <algorithm>
#include <set>
#include <random>
#include <unittest++/UnitTest++.h>
#include "unit_test_util.h"
#include "m1.h"
#include "StreetsDatabaseAPI.h"

using ece297test::relative_error;
struct MapFixture {
    MapFixture() {
        //Load the map
        load_map("/cad2/ece297s/public/maps/toronto_canada.streets.bin");

        //Initialize random number generators
        rng = std::minstd_rand(10);
        rand_street = std::uniform_int_distribution<unsigned>(1, getNumberOfStreets()-1);
    }

    ~MapFixture() {
        //Clean-up
        close_map();
    }
    
    std::minstd_rand rng;
    std::uniform_int_distribution<unsigned> rand_street;
};
SUITE(street_names_from_ids_public_toronto_canada) {
    TEST_FIXTURE(MapFixture, Find_Street_Ids_From_Name_UnitTest){
    
        std::vector<unsigned> expected;
        std::vector<unsigned> actual;

        expected = {21221};
        std::vector<unsigned> result;
        actual = find_street_ids_from_name("Louvain Avenue");
        std::sort(actual.begin(), actual.end());
        CHECK_EQUAL (expected, actual); //compare the ids   

        std::vector<std::string> street_names_1;
        for(size_t i = 0; i < 1000000; i++) {
            unsigned street_id1 = rand_street(rng);
            street_names_1.push_back(getStreetName(street_id1));
        }
        {
            //Timed Test
            ECE297_TIME_CONSTRAINT(250);
            for(size_t i = 0; i < 1000000; i++) {
                result = find_street_ids_from_name(street_names_1[i]);
            }
            ECE297_TIME_CONSTRAINT(0.000250);
            for(size_t i = 0; i < 1; i++) {
                result = find_street_ids_from_name(street_names_1[i]);
            }
        }
    }
}
