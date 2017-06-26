#include "testcase.hpp"
#include <ketchup.hpp>
#include <bfgrid.hpp>
#include <iostream>

struct InvertTest: TestCase {
    InvertTest() : TestCase( "invert" ) {}
    void run() {
        std::cout << "North: " << invert( Pred::North ) << "\n";
        std::cout << "West: " << invert( Pred::West ) << "\n";
        std::cout << "South: " << invert( Pred::South ) << "\n";
        std::cout << "East: " << invert( Pred::East ) << "\n";
    }

};

static InvertTest _test;