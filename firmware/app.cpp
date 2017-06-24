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

#include "ev3cxx.h"
#include "app.h"

#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h> 

#include "json11.hpp"

#include "Robot.h"
#include "Detector.h"
#include "libs/logging/logging.hpp"
#include "libs/logging/FileLogSink.h"
#include "libs/logging/DisplayLogSink.h"
#include "libs/logging/BTLogSink.h"


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

void waitForSomething(ev3cxx::BrickButton& btn, ev3cxx::Bluetooth& bt, std::string msg, bool skip = false) {
    if(!skip) {
        display.format("% ") % msg;
        format(bt,"% ") % msg;
        while(!btn.isPressed()) {
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
void main_task(intptr_t unused) {
    bool btnSkip = false;
    //Robot.Debug debugInfo = Robot.Debug::No;
    RobotGeometry robotGeometry{55, 125};

    ev3cxx::Bluetooth bt{true};

    ev3cxx::ColorSensor colorL{ev3cxx::SensorPort::S1};
    ev3cxx::ColorSensor colorR{ev3cxx::SensorPort::S4};
    ev3cxx::TouchSensor touchStop{ev3cxx::SensorPort::S3};
    ev3cxx::BrickButton btnEnter(ev3cxx::BrickButtons::ENTER);
    ev3cxx::BrickButton btnStop(ev3cxx::BrickButtons::UP);

    ev3cxx::Motor motorL{ev3cxx::MotorPort::B};
    ev3cxx::Motor motorR{ev3cxx::MotorPort::C};
    ev3cxx::MotorTank motors{ev3cxx::MotorPort::B, ev3cxx::MotorPort::C};

    Logger l;
    l.addSink(ALL, std::unique_ptr<LogSink>(new FileLogSink("log.txt", 80)));
    l.addSink(ALL, std::unique_ptr<LogSink>(new DisplayLogSink(display, 15, 16)));
    l.addSink(ALL, std::unique_ptr<LogSink>(new BTLogSink(bt, 80)));

    json11::Json config = load_config("config.json");
    display_intro(config, display);

    display.setFont(EV3_FONT_SMALL);
    display.format(" \n Press ENTER to begin\n");
    display.setFont(EV3_FONT_MEDIUM);
    
    while(!btnEnter.isPressed()) {
        ev3cxx::delayMs(10);
    }
    
    char welcomeString[] = "\r\tK-ranka 2021\nev3cxx-ketchup\nInitialization...\n";
    format(bt, "\n\n% ") % welcomeString;
    display.format("% ") % welcomeString;
    Robot robot{robotGeometry, colorL, colorR, touchStop, btnEnter, btnStop, motors, 
        bt, Robot::Debug(Robot::Debug::Text | Robot::Debug::Packet)};
    robot.init();

    waitForSomething(btnEnter, bt, "Cal => ENTER\n", true);
    robot.calibrateSensor();

    while(true) {
        //waitForSomething(btnEnter, bt, "Start => ENTER\n", false);
        bool skip = false;
        std::string msg = "Start => ENTER\n";
        if(!skip) {
            display.format("% ") % msg.c_str();
            format(bt,"% ") % msg.c_str();
            while(!btnEnter.isPressed()) {
                if(robot.debugState() & Robot::Debug::Packet)
                    robot.packetColorReflected();
                ev3cxx::delayMs(10);
            }
        }

        robot.step(2);
        format(bt, "\n\n\r");
        
        for(int i = 0; i < 2; i++) {
            robot.rotate(-90);//, robot.Debug::Text + robot.Debug::Packet);
            robot.step(1);
            robot.rotate(-90);
            robot.step(1);
            robot.rotate(-90);
            robot.step(1);
        }
        motors.off(false);
    }
}
