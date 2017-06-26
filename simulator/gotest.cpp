#include "testcase.hpp"
#include <ketchup.hpp>

struct GoTest: TestCase {
    GoTest() : TestCase( "go" ) {}

    void run() {
        plan.clearScreen();
        Robot robot( [&] {
            plan.clear();
            plan.drawGrid();
            for ( auto k : robot.ketchups )
                plan.drawKetchup( k );
            plan.drawEnemy( robot.enemy );
            plan.drawRobot( robot.position );
            plan.show();
            robot.dump();
            std::this_thread::sleep_for( 600ms );
            plan.clearScreen();
        } );
        robot.position = { 0, 3, Pred::East };
        robot.ketchups = { { 3, 1 }, { 6, 4 } };

        KetchupLogic controller( robot );
        controller.go( { 4, 1 } );
        robot.enemy = { 5, 1 };
        robot.yield();
        controller.go( { 6, 1 } );
        controller.go( { 6, 6 } );
    }
};

static GoTest _test;
