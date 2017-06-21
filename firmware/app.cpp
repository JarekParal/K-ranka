/**
 * This is sample program for testing two Color sensors in EV3RT C++ API.
 *
 * Author: Jaroslav Páral (jarekparal)
 */

#include <cstdlib>

#include "ev3cxx.h"
#include "app.h"

#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h> 

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

void main_task(intptr_t unused) {
    ev3cxx::Bluetooth bt{true};

    char welcomeString[] = "\r\tEV3RT ev3cxx \n\tColorSen2x\nInitialization...\n";

    format(bt, "\n\n% ") % welcomeString;
    display.format("% ") % welcomeString;

    ev3cxx::ColorSensor colorL{ev3cxx::SensorPort::S1};
    ev3cxx::ColorSensor colorR{ev3cxx::SensorPort::S4};
    ev3cxx::BrickButton btnEnter(ev3cxx::BrickButtons::ENTER);
    ev3cxx::BrickButton btnStop(ev3cxx::BrickButtons::BACK);

    ev3cxx::Motor motorL{ev3cxx::MotorPort::B};
    ev3cxx::Motor motorR{ev3cxx::MotorPort::C};
    ev3cxx::MotorTank motors{ev3cxx::MotorPort::B, ev3cxx::MotorPort::C};

    AvakarPacket packetOut;
    int lVal, rVal, lMin, lMax, rMin, rMax;
    RollingAverage<int, 3> lAvgVal, rAvgVal;

    lMin = rMin = 100;
    lMax = rMax = 0;
    
    int error;
    const int threshold = 70;

    int errorLine = 0;
    const int errorLineCorrection = 100;
    int startSpeed = 20;
    int motorLSpeed, motorRSpeed;

    display.format("Calibration start\n");
    while(!btnEnter.isPressed()) {
        lVal = colorL.reflected();
        rVal = colorR.reflected();
        calibrateSensor(lVal, lMin, lMax);
        calibrateSensor(rVal, rMin, rMax);
        ev3cxx::delayMs(50);

        display.format("\r%3:%3  %3:%3") % lMin % lMax % rMin % rMax;
        format(bt, "lMin:%3  lMax:%3\t") % lMin % lMax;
        format(bt, "rMin:%3  rMax:%3\n") % rMin % rMax; 
    }
    display.format("Calibration stop\n");

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

        packetOut.set_command(0);
        // packetOut.push<uint8_t>(lVal);
        // packetOut.push<uint8_t>(rVal);
        // packetOut.push<uint8_t>(lVal + rVal);
        packetOut.push<uint8_t>(lCalVal);
        packetOut.push<uint8_t>(rCalVal);
        packetOut.push<uint8_t>(lCalVal + rCalVal);
        packetOut.push<uint8_t>(error);
        for(char ch: packetOut) {
           bt.write(ch); 
        }
        packetOut.clear();

        errorLine = (lCalVal + rCalVal) - errorLineCorrection;
        errorLine /= 4;

        if(lCalVal > rCalVal)
            errorLine *= -1;

        motorLSpeed = startSpeed - errorLine;
        motorRSpeed = startSpeed + errorLine;

        motors.on(motorRSpeed, motorLSpeed);
        display.format("\r  L%2 R %2 E%2") % motorLSpeed % motorRSpeed % errorLine;

        packetOut.set_command(1);
        packetOut.push<int16_t>(motorLSpeed);
        packetOut.push<int16_t>(motorRSpeed);
        packetOut.push<int16_t>(errorLine);
        for(char ch: packetOut) {
           bt.write(ch); 
        }
        packetOut.clear();

        ev3cxx::delayMs(5);
    }
    motors.off(false);

}
