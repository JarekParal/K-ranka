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
    display.resetScreen();
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


char welcomeString[] = "\r\tK-ranka 2017\nev3cxx-ketchup\nInitialization...\n";

void main_task(intptr_t unused) {
    //Robot.Debug debugInfo = Robot.Debug::No;
    RobotGeometry robotGeometry{55, 125};

    ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);

    ev3cxx::Bluetooth bt{true};
    Logger l;
    l.addSink(ALL, std::unique_ptr<LogSink>(new FileLogSink("log.txt", 80)));
    l.addSink(ALL, std::unique_ptr<LogSink>(new DisplayLogSink(display, 15, 16)));
    l.addSink(ALL, std::unique_ptr<LogSink>(new BTLogSink(bt, 80)));

    ev3cxx::ColorSensor colorL{ev3cxx::SensorPort::S1};
    l.logInfo("Sensor", "colorL - init");
    ev3cxx::ColorSensor colorR{ev3cxx::SensorPort::S4};
    l.logInfo("Sensor", "colorR - init");
    ev3cxx::TouchSensor ketchupSensor{ev3cxx::SensorPort::S3};
    l.logInfo("Sensor", "ketchupS - init");
    ev3cxx::UltrasonicSensor sonar{ev3cxx::SensorPort::S2};
    l.logInfo("Sensor", "sonar - init");
    ev3cxx::BrickButton btnEnter(ev3cxx::BrickButtons::ENTER);
    ev3cxx::BrickButton btnStop(ev3cxx::BrickButtons::UP);
    l.logInfo("Sensor", "btn - init");

    ev3cxx::Motor motorL{ev3cxx::MotorPort::B};
    ev3cxx::Motor motorR{ev3cxx::MotorPort::C};
    ev3cxx::MotorTank motors{ev3cxx::MotorPort::B, ev3cxx::MotorPort::C};
    ev3cxx::Motor motorGate{ev3cxx::MotorPort::D};

    json11::Json config = load_config("config.json");
    //display_intro(config, display);

    // format(bt, "\n\n% ") % welcomeString;
    // display.format("% ") % welcomeString;
    l.logInfo("INFO", "{}") << welcomeString;

    Robot robot{robotGeometry, colorL, colorR, ketchupSensor, btnEnter, btnStop, motors, motorGate,
        l, bt, sonar, Robot::Debug(Robot::Debug::Text | Robot::Debug::Packet)};
    robot.ledRed();

    // waitForButton(btnEnter, l, "Testing components", false, [&]{
    //     l.logDebug("TESTING", "ketchupSensor: {}") << ketchupSensor.isPressed();
    //     ev3cxx::delayMs(50);
    // });

    while(!btnEnter.isPressed()) {
        l.logDebug("TESTING", "ketchupSen: {} ", int(ketchupSensor.isPressed()));
        ev3cxx::delayMs(200);
        if(!ketchupSensor.isPressed())
            robot.ledOrange();
        else
            robot.ledGreen();
    }
    robot.ledRed();

    l.logInfo("ROBOT", "init() - start");
    robot.init();
    l.logInfo("ROBOT", "init() - end");

    robot.calibrateSensor();

    robot.ledGreen();

    Robot::State robotState;

    while(true) {
        waitForButton(btnEnter, l, "Run => ENTER", false);
        robotState = robot.step(1);
        l.logInfo("MOVE", "step() => {}") << int(robotState);
        //l.logInfo("MOVE", "step() => {}") << Robot::StateStr[int(robotState)];
        if(robotState == Robot::State::RivalDetected) {
            motors.off(true);
            robot.rotate( 90 );
        }
        motors.off(false);
    }
}
