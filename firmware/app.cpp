/**
 * Robot K-ranka program for the Ketchup House competition (Robotics Day 2017).
 *
 * Author: Jan Mrázek (yaqwsx), Martin Mikšík (mamiksik), Jaroslav Páral (jarekparal)
 */

#include <cstdlib>
#include <cmath>
#include <string>
#include <fstream>
#include <streambuf>
#include <functional>
#include <memory>

#include "ev3cxx.h"
#include "app.h"

#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h>

#include "json11.hpp"

#include "libs/logging/logging.hpp"
#include "libs/logging/FileLogSink.h"
#include "libs/logging/DisplayLogSink.h"
#include "libs/logging/BTLogSink.h"

#include "Robot.h"
#include "Detector.h"
#include "ketchup.hpp"

extern "C" void __sync_synchronize() {};

using ev3cxx::display;
using ev3cxx::format;

void calibrateSensor(int val, int& min, int& max) {
    if(max < val)
        max = val;
    if(min > val)
        min = val;
}

int getCalibratedValue(int val, int vMin, int vMax, int min, int max, bool clamp = false) {
    int result = ((val - vMin) * (float(max - min) / (vMax - vMin))) + min;
    if(clamp) {
        if(min > result)
            return min;
        if(max < result)
            return max;
    }
    return result;
}

void waitForButton(ev3cxx::BrickButton& btn, Logger& l, std::string msg, bool skip = false,
                   std::function<void()> idleTask = []{})
{
    if(!skip) {
         l.logInfo("INFO", "{}") << msg;
        while(!btn.isPressed()) {
            idleTask();
            ev3cxx::delayMs(10);
        }
    }
}

void display_intro(json11::Json config, ev3cxx::detail::Display &display){
    json11::Json welcome = config["welcome"].object_items();
    display.format("\n \n \n");
    display.format(" % \n") % welcome["robo"].string_value();
    display.format(" % ") % welcome["web"].string_value();
    ev3cxx::delayMs(1500);
    display.resetScreen();
    display.format("\n \n \n");
    display.format("Name: % \n") % welcome["name"].string_value();
    display.setFont(EV3_FONT_SMALL);
    display.format("Version: % \n") % welcome["version"].string_value();
    display.format("Authors: % \n")  % welcome["authors"].string_value();
    display.setFont(EV3_FONT_MEDIUM);
    ev3cxx::delayMs(1500);
}

json11::Json load_config(std::string fileName){
    std::ifstream configFile(fileName);
    std::string configJson((std::istreambuf_iterator<char>(configFile)),
                 std::istreambuf_iterator<char>());

    std::string ErrorMsg = std::string("Json parsing error");
    return json11::Json::parse(configJson, ErrorMsg);
}

Logger l;
char welcomeString[] = "\r\tK-ranka 2017\nev3cxx-ketchup\nInitialization...\n";

void main_task(intptr_t unused) {
    display.resetScreen();
    display.format(" % \n") % welcomeString;
    //robot->Debug debugInfo = robot->Debug::No;
//     RobotGeometry robotGeometry{55, 125};
//     RobotGeometry robotGeometry{55, 125};
    RobotGeometry robotGeometry{55, 132};

    ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);

    ev3cxx::Bluetooth bt{true};
    //l.addSink(ALL, std::unique_ptr<LogSink>(new FileLogSink("log.txt", 80)));
    l.addSink(ALL, std::unique_ptr<LogSink>(new DisplayLogSink(display, 15, 16)));
    //l.addSink(ALL, std::unique_ptr<LogSink>(new BTLogSink(bt, 80)));
    

    ev3cxx::ColorSensor colorL{ev3cxx::SensorPort::S3};
    l.logInfo("Sensor", "colorL - init");
    ev3cxx::ColorSensor colorR{ev3cxx::SensorPort::S4};
    l.logInfo("Sensor", "colorR - init");
    ev3cxx::TouchSensor ketchupSensor{ev3cxx::SensorPort::S2};
    l.logInfo("Sensor", "ketchupS - init");
    ev3cxx::UltrasonicSensor sonar{ev3cxx::SensorPort::S1};
    l.logInfo("Sensor", "sonar - init");
    ev3cxx::BrickButton btnEnter(ev3cxx::BrickButtons::ENTER);
    ev3cxx::BrickButton btnStop(ev3cxx::BrickButtons::UP);
    l.logInfo("Sensor", "btn - init");

    ev3cxx::Motor motorL{ev3cxx::MotorPort::D};
    ev3cxx::Motor motorR{ev3cxx::MotorPort::A};
    ev3cxx::MotorTank motors{ev3cxx::MotorPort::D, ev3cxx::MotorPort::A};
    
    ev3cxx::Motor motorGate{ev3cxx::MotorPort::C, ev3cxx::MotorType::MEDIUM};

    ev3cxx::Motor motorSensor{ev3cxx::MotorPort::B, ev3cxx::MotorType::MEDIUM};
    motorSensor.off();

    json11::Json config = load_config("config.json");
    display_intro(config, display);

    // format(bt, "\n\n% ") % welcomeString;
    // display.format("% ") % welcomeString;
    l.logInfo("INFO", "{}") << welcomeString;

    auto robot = std::make_unique< Robot >(robotGeometry, colorL, colorR, ketchupSensor, btnEnter, btnStop, motors, motorGate,
        l, bt, sonar, motorSensor, Robot::Debug(Robot::Debug::Text | Robot::Debug::Packet));
    auto controller = std::make_unique< KetchupLogic >( *robot );
    robot->ledRed();

    // waitForButton(btnEnter, l, "Testing components", false, [&]{
    //     l.logDebug("TESTING", "ketchupSensor: {}") << ketchupSensor.isPressed();
    //     ev3cxx::delayMs(50);
    // });

    ev3cxx::delayMs( 2000 );

    l.logInfo("ROBOT", "init() - start");
    robot->init();
    l.logInfo("ROBOT", "init() - end");

    //l.logInfo("ROBOT", "calibrate() - start");
    robot->calibrateSensor();
    //l.logInfo("ROBOT", "calibrate() - end");
    while(!btnEnter.isPressed()) {
        ev3cxx::delayMs(200);
        if(!ketchupSensor.isPressed())
            robot->ledOrange();
        else
            robot->ledGreen();
    }
    robot->ledRed();
    ev3cxx::delayMs( 1000 );

    robot->ledGreen();
    Robot::State robotState;
    //int tinCnt = 0;

//    while(true){
//        robot->rotate(90);
//        waitForButton(btnEnter, l, "Run => ENTER", false);
//    }

    // while(false) {
     waitForButton(btnEnter, l, "Run => ENTER", false);
//    display.resetScreen();
        // robotState = robot->step(1);
        // robot->rotate( 90 );
        // l.logInfo("MOVE", "step() => {}") << int(robotState);
        //l.logInfo("MOVE", "step() => {}") << Robot::StateStr[int(robotState)];
        // if(robotState == Robot::State::KetchupDetected) {
        //     tinCnt++;
        // }
        // if(tinCnt == 1) {
        //     robot->gateOpen();
        //     break;
        // }
        // motors.off(false);
    // }

//         controller->go( { 3, 0 } );
//        controller->go( { 2, 6 } );
//        controller->go( { 2, 0 } );

//    controller->go( { 0, 0 } );
//    controller->go( { 1, 1 } );

	controller->go( { 0, 3 } );
	controller->go( { 6, 3 } );


//	controller->go( { 6, 3 } );
//	controller->go( { 0, 3 } );


//    robot->step();
//    controller->go( { 1, 2 } );
//        controller->go( { 3, 3 } );
//        controller->go( { 3, 0 } );
//        controller->go( { 3, 6 } );
    // controller->go( { 2, 6 } );
    // controller->go( { 2, 6 } );
    // controller->go( { 2, 1 } );
    // controller->unload();


    // robot->rotate( 90 );
    motors.off(true);
}
