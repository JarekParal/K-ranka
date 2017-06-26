#include <iostream>
#include <string>
#include <thread>
#include <map>
#include <cassert>
#include <bfgrid.hpp>
#include "testcase.hpp"

int main( int argc, char **argv ) {
    if ( argc == 1 ) {
        TestCase::runAll();
        return 0;
    }
    if ( argc == 2 ) {
        std::string arg( argv[ 1 ] );
        if ( arg == "list" ) {
            TestCase::listCases();
            return 0;
        }
        TestCase::run( arg );
        return 0;
    }
    std::cerr << "Invalid usage!\n";
    return 1;
}