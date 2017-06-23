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

void packet_send_color_sensors (ev3cxx::Bluetooth & bt, int lCalVal, int rCalVal, int error) {
    AvakarPacket packetOut;

    packetOut.set_command(0);
    // packetOut.push<uint8_t>(lVal);
    // packetOut.push<uint8_t>(rVal);
    // packetOut.push<uint8_t>(lVal + rVal);
    packetOut.push<uint8_t>(lCalVal);
    packetOut.push<uint8_t>(rCalVal);
    packetOut.push<uint8_t>(lCalVal + rCalVal);
    packetOut.push<int8_t>(error);
    for(char ch: packetOut) {
        bt.write(ch); 
    }
    packetOut.clear();
}

void packet_send_motors_line(ev3cxx::Bluetooth & bt, int motorLSpeed, int motorRSpeed, int errorLine) {
    AvakarPacket packetOut;

    packetOut.set_command(1);
    packetOut.push<int16_t>(motorLSpeed);
    packetOut.push<int16_t>(motorRSpeed);
    packetOut.push<int16_t>(errorLine);
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
}

json11::Json load_config(std::string fileName){
    std::ifstream configFile(fileName);
    std::string configJson((std::istreambuf_iterator<char>(configFile)),
                 std::istreambuf_iterator<char>());
    
    std::string ErrorMsg = std::string("Json parsing error");
    return json11::Json::parse(configJson, ErrorMsg);
    //json11::Json config = json11::Json::parse(configJson, ErrorMsg);
}

void main_task(intptr_t unused) {
    ev3cxx::Bluetooth bt{true};

    ev3cxx::ColorSensor colorL{ev3cxx::SensorPort::S1};
    ev3cxx::ColorSensor colorR{ev3cxx::SensorPort::S4};
    ev3cxx::BrickButton btnEnter(ev3cxx::BrickButtons::ENTER);
    ev3cxx::BrickButton btnStop(ev3cxx::BrickButtons::UP);

    ev3cxx::Motor motorL{ev3cxx::MotorPort::B};
    ev3cxx::Motor motorR{ev3cxx::MotorPort::C};
    ev3cxx::MotorTank motors{ev3cxx::MotorPort::B, ev3cxx::MotorPort::C};

    int lVal, rVal, lMin, lMax, rMin, rMax;
    RollingAverage<int, 3> lAvgVal, rAvgVal;

    lVal = colorL.reflected();
    rVal = colorR.reflected();

    lMin = rMin = 100;
    lMax = rMax = 0;
    
    int error;
    const int threshold = 70;

    int errorLine = 0;
    const int errorLineCorrection = 100;
    int startSpeed = 20;
    int motorLSpeed, motorRSpeed;

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
    motors.onForDegrees(10, -10, -(motorsCalibrationDegrees/2), false, true);
    motors.leftMotor().resetPosition();
    motors.rightMotor().resetPosition();
    motors.onForDegrees(10, -10, motorsCalibrationDegrees, true, false);
    
    while(motors.leftMotor().degrees() < (motorsCalibrationDegrees - 10)) {
    //while(motorsCalibrationDegrees == approximately(motors.leftMotor().degrees() , 20)) {
        lVal = colorL.reflected();
        rVal = colorR.reflected();
        calibrateSensor(lVal, lMin, lMax);
        calibrateSensor(rVal, rMin, rMax);
        ev3cxx::delayMs(10);

        format(bt, "mL:%4 mR:%4  ") % motors.leftMotor().degrees() % motors.rightMotor().degrees(); 

        display.format("\r%3:%3  %3:%3") % lMin % lMax % rMin % rMax;
        format(bt, "lMin:%3  lMax:%3\t") % lMin % lMax;
        format(bt, "rMin:%3  rMax:%3\n") % rMin % rMax; 
    }
    ev3cxx::delayMs(1000);
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
            lVal = colorL.reflected();
            rVal = colorR.reflected();
            int lCalVal = getCalibratedValue(lVal, lMin, lMax, 0, 100, true);
            int rCalVal = getCalibratedValue(rVal, rMin, rMax, 0, 100, true);

            // //HACK
            // lCalVal = lVal;
            // rCalVal = rVal;

            lAvgVal.push(lCalVal);
            rAvgVal.push(rCalVal);

            lCalVal = lAvgVal.get_average();
            rCalVal = rAvgVal.get_average();

            //display.format("\rL%3:%3 R%3:%3") % lVal % lCalVal % rVal % rCalVal;
            
            if(rCalVal > threshold || lCalVal > threshold)
                error = 100;
            else
                error = rCalVal - lCalVal;

            // Hack 2
            error = rCalVal - lCalVal;

            packet_send_color_sensors(bt, lCalVal, rCalVal, error);

            errorLine = (lCalVal + rCalVal) - errorLineCorrection;
            errorLine /= 4;

            if(lCalVal > rCalVal)
                errorLine *= -1;

            // Hack 3
            errorLine = error / 8;

            motorLSpeed = startSpeed + errorLine;
            motorRSpeed = startSpeed - errorLine;

            motors.on(motorRSpeed, motorLSpeed);
            display.format("\r  L%2 R %2 E%2") % motorLSpeed % motorRSpeed % errorLine;

            //packet_send_motors_line(bt, motorLSpeed, motorRSpeed, errorLine);

            ev3cxx::delayMs(5);
        }
        motors.off(false);
    }
}
