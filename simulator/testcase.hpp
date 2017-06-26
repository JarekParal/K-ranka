#pragma once

#include "simulator.hpp"
#include <iostream>
#include <string>
#include <map>
#include <chrono>

using namespace std::chrono_literals;

namespace {
    static const int POINTS = 7;
}

struct TestCase {
    TestCase( std::string name )
        : frameBuf(
            { ( POINTS - 1 ) * 3 + 1, ( POINTS - 1 ) * 2 + 1 }, L" " ),
          plan( { POINTS, POINTS } )
    {
        cases()[ name ] = this;
    }

    virtual void run() = 0;

    static std::map< std::string, TestCase *>& cases() {
        static std::map< std::string, TestCase *> cases;
        return cases;
    }

    static void listCases() {
        for ( const auto& x : cases() ) {
            std::cout << x.first << "\n";
        }
    }

    static void runAll() {
        for ( auto& x : cases() ) {
            std::cout << "Running: " << x.first << "\n";
            x.second->run();
            std::cout << "Done: " << x.first << "\n";
        }
    }

    static void run( std::string name ) {
        auto x = cases().find( name );
        if ( x == cases().end() ) {
            std::cerr << "Cannot find test case " << name << "\n";
            return;
        }
        x->second->run();
    }

    FrameBuffer frameBuf;
    GamePlan plan;
};