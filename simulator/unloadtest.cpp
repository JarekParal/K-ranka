#include "testcase.hpp"
#include <ketchup.hpp>

struct UnloadTest: TestCase {
    UnloadTest() : TestCase( "unload" ) {}

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
            std::wcout << "Inside: " << ctrl->ketchupCount << "\n";
            std::this_thread::sleep_for( 500ms );
            plan.clearScreen();
        } );
        robot.position = { 0, 3, Pred::East };
        for ( int x = 3; x != 5; x++ ) {
            for ( int y = 1; y != 6; y++ )
                robot.ketchups.insert( { x, y } );
        }

        KetchupLogic controller( robot );
        ctrl = &controller;
        controller.go( { 6, 4 } );
        controller.go( { 6, 3 } );
        controller.go( { 2, 4 } );

//        controller.go( { 1, 1 } );
        controller.unload();
    }

    KetchupLogic *ctrl;
};

static UnloadTest _test;