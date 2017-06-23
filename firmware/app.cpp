/**
 * This is sample program for testing two Color sensors in EV3RT C++ API.
 *
 * Author: Jaroslav Páral (jarekparal)
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

#include "Detector.h"
#include "libs/logging/logging.hpp"
#include "libs/logging/FileLogSink.h"
#include "libs/logging/DisplayLogSink.h"
#include "libs/logging/BTLogSink.h"

using ev3cxx::display;
using ev3cxx::format;
using atoms::AvakarPacket;
using atoms::RollingAverage;

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

void packet_send_color_sensors (ev3cxx::Bluetooth & bt, int lCalVal, int rCalVal, int errorNeg) {
    AvakarPacket packetOut;

    packetOut.set_command(0);
    // packetOut.push<uint8_t>(lVal);
    // packetOut.push<uint8_t>(rVal);
    // packetOut.push<uint8_t>(lVal + rVal);
    packetOut.push<uint8_t>(lCalVal);
    packetOut.push<uint8_t>(rCalVal);
    packetOut.push<uint8_t>(lCalVal + rCalVal);
    packetOut.push<int8_t>(errorNeg);
    for(char ch: packetOut) {
        bt.write(ch); 
    }
    packetOut.clear();
}

void packet_send_motors_line(ev3cxx::Bluetooth & bt, int motorLSpeed, int motorRSpeed, int errorNegLine) {
    AvakarPacket packetOut;

    packetOut.set_command(1);
    packetOut.push<int16_t>(motorLSpeed);
    packetOut.push<int16_t>(motorRSpeed);
    packetOut.push<int16_t>(errorNegLine);
    for(char ch: packetOut) {
        bt.write(ch); 
    }
    packetOut.clear();
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
    ev3cxx::Bluetooth bt{true};

    ev3cxx::ColorSensor colorL{ev3cxx::SensorPort::S1};
    ev3cxx::ColorSensor colorR{ev3cxx::SensorPort::S4};
    ev3cxx::TouchSensor touchStop{ev3cxx::SensorPort::S3};
    ev3cxx::BrickButton btnEnter(ev3cxx::BrickButtons::ENTER);
    ev3cxx::BrickButton btnStop(ev3cxx::BrickButtons::UP);

    ev3cxx::Motor motorL{ev3cxx::MotorPort::B};
    ev3cxx::Motor motorR{ev3cxx::MotorPort::C};
    ev3cxx::MotorTank motors{ev3cxx::MotorPort::B, ev3cxx::MotorPort::C};

    int lCalVal, rCalVal;
    RollingAverage<int, 3> lAvgVal, rAvgVal;

    // Init ColorSensor:
    // colorL.reflected() is blocking function 
    // => If the sensor is not connected then the program freezes
    colorL.reflected();
    colorR.reflected();
    
    int errorNeg, errorPos;
    const int errorPosThreshold = 80;

    int errorLine = 0;
    int startSpeed = 20;
    int motorLSpeed, motorRSpeed;
    int targetDistance = -1;

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
    const int motorsCalibrationDegrees = 200;
    display.format("Calibration start\n");
    motors.onForDegrees(-10, 10, motorsCalibrationDegrees/2, false, true);
    motors.leftMotor().resetPosition();
    motors.rightMotor().resetPosition();
    motors.onForDegrees(10, -10, motorsCalibrationDegrees, true, false);
    
    while(motors.leftMotor().degrees() < (motorsCalibrationDegrees - 10)) {
    //while(motorsCalibrationDegrees == approximately(motors.leftMotor().degrees() , 20)) {
        ev3cxx::delayMs(10);

        colorL.calibrateReflection();
        colorR.calibrateReflection();

        format(bt, "mL:%4 mR:%4  ") % motors.leftMotor().degrees() % motors.rightMotor().degrees(); 

        display.format("\r%3:%3  %3:%3") % colorL.min() % colorL.max() % colorR.min() % colorR.max();
        format(bt, "lMin:%3  lMax:%3\t") % colorL.min() % colorL.max();
        format(bt, "rMin:%3  rMax:%3\n") % colorR.min() % colorR.max(); 
    }
    ev3cxx::delayMs(500);
    motors.leftMotor().resetPosition();
    motors.rightMotor().resetPosition();
    motors.onForDegrees(-10, 10, motorsCalibrationDegrees/2, false, true);
    display.format("Calibration stop\n");

    // Start line follower
    while(true) {
        display.format("Start => ENTER\n");
        while(!btnEnter.isPressed()) {
            ev3cxx::delayMs(10);
        }

        while(!btnStop.isPressed()) {
            lCalVal = colorL.reflected(true, true);
            rCalVal = colorR.reflected(true, true);

            lAvgVal.push(lCalVal);
            rAvgVal.push(rCalVal);

            lCalVal = lAvgVal.get_average();
            rCalVal = rAvgVal.get_average();

            errorNeg = rCalVal - lCalVal;
            errorPos = rCalVal + lCalVal;

            packet_send_color_sensors(bt, lCalVal, rCalVal, errorNeg);

            errorLine = errorNeg / 12;

            motorLSpeed = startSpeed + errorLine;
            motorRSpeed = startSpeed - errorLine;

            if(touchStop.isPressed()) {
                motors.off();
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
            } else if(targetDistance < motors.leftMotor().degrees() 
                   && errorPos < errorPosThreshold) {
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);
                
                motors.off();
                ev3cxx::delayMs(1000);
                targetDistance = motors.leftMotor().degrees() + 180;
            } else {
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::GREEN);
                motors.on(motorRSpeed, motorLSpeed);
            }

            display.format("\r  L%2 R %2 E%2") % motorLSpeed % motorRSpeed % errorLine;

            //packet_send_motors_line(bt, motorLSpeed, motorRSpeed, errorNegLine);

            ev3cxx::delayMs(5);
        }
        motors.off(false);
    }
}
