/**
 * This is sample program for testing two Color sensors in EV3RT C++ API.
 *
 * Author: Jaroslav Páral (jarekparal)
 */

#include <cstdlib>
#include <cmath>

#include "ev3cxx.h"
#include "app.h"


#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h> 

//#include "Detector.h"

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

void main_task(intptr_t unused) {
    ev3cxx::Bluetooth bt{true};

    char welcomeString[] = "\r\tK-ranka \nev3cxx-ketchup\nInitialization...\n";

    format(bt, "\n\n% ") % welcomeString;
    display.format("% ") % welcomeString;

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
    
    int errorNeg, errorPos;
    const int errorPosThreshold = 80;
    const int errorStopWithoutLineThres = 20;
    const int threshold = 70;

    int errorLine = 0;
    const int errorNegLineCorrection = 100;
    int startSpeed = 20;
    int motorLSpeed, motorRSpeed;
    int targetDistance = -1;

    // Calibration
    display.format("Cal => ENTER\n");
    format(bt,"Cal => ENTER\n");
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
                errorNeg = 100;
            else
                errorNeg = rCalVal - lCalVal;

            // Hack 2
            errorNeg = rCalVal - lCalVal;
            errorPos = rCalVal + lCalVal;

            packet_send_color_sensors(bt, lCalVal, rCalVal, errorNeg);

            errorLine = (lCalVal + rCalVal) - errorNegLineCorrection;
            errorLine /= 4;

            if(lCalVal > rCalVal)
                errorLine *= -1;

            // Hack 3
            errorLine = errorNeg / 12;

            motorLSpeed = startSpeed + errorLine;
            motorRSpeed = startSpeed - errorLine;

            if(targetDistance < motors.leftMotor().degrees() && errorPos < errorPosThreshold) {
            //     format(bt, "\r\n\nlCalVal:%4  rCalVal:%4  errorPos:%5") % lCalVal % rCalVal % 200;
            //     format(bt, "\r\n\nlCalVal:%4  rCalVal:%4  errorPos:%5") % lCalVal % rCalVal % 200
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);
                motors.on(0, 0);
                ev3cxx::delayMs(1000);
                //motors.onForDegrees(startSpeed, startSpeed, 360);
                targetDistance = motors.leftMotor().degrees() + 180;
            } 
            // else if((lCalVal < errorStopWithoutLineThres) && (rCalVal < errorStopWithoutLineThres)) {
            //     motors.on(0, 0);
            //     ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
            // } 
            else {
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
