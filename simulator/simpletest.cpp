#include "testcase.hpp"

struct SimpleTestCase: TestCase {
    SimpleTestCase(): TestCase("SimpleTestCase") {}

    void run() {
        Robot robot( [&] {
            plan.drawGrid();
            plan.drawRobot( robot.position );
            plan.show();
            robot.dump();
            std::this_thread::sleep_for( 200ms );
            robot.clearDump();
            plan.clearScreen();
        });
        robot.position.orient = Pred::West;

        for( int i = 0; i != 2; i++ ) {
            for ( int i = 0; i != 4; i++ ) {
                robot.rotate( -90 );
                robot.step( POINTS - 1 );
            }
        }
    }
};

static SimpleTestCase _testcase;