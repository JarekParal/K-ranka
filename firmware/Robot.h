
#pragma once

#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h> 

#include "ev3cxx.h"

#include "RobotGeometry.h"
#include "DifferentialDrive.h"

using ev3cxx::display;

void packet_send_color_sensors (ev3cxx::Bluetooth & bt, int lCalVal, int rCalVal, int errorNeg) {
    atoms::AvakarPacket packetOut;

    packetOut.set_command(0);
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
    atoms::AvakarPacket packetOut;

    packetOut.set_command(1);
    packetOut.push<int16_t>(motorLSpeed);
    packetOut.push<int16_t>(motorRSpeed);
    packetOut.push<int16_t>(errorNegLine);
    for(char ch: packetOut) {
        bt.write(ch); 
    }
    packetOut.clear();
}

class Robot {
public:
    enum class State {
        PositionReached,
        RivalDetected,
        StopBtnPressed
    };

    enum Debug {
        No = 1 << 0,
        Text = 1 << 1,
        Packet = 1 << 2,
        Default = 1 << 3,
        TextPacket = Text | Packet
    };

    enum class Direction {
        Left = -1,
        Right = 1
    };

    Robot(RobotGeometry& rGeometry, ev3cxx::ColorSensor& ColorL, ev3cxx::ColorSensor& ColorR, ev3cxx::TouchSensor& TouchStop,
    ev3cxx::BrickButton& BtnEnter, ev3cxx::BrickButton& BtnStop, ev3cxx::MotorTank& Motors, 
    Logger& Log, ev3cxx::Bluetooth& Bt, Debug DebugGlobal = Debug::No)
    : robotGeometry(rGeometry),
    colorL(ColorL), colorR(ColorR), touchStop(TouchStop), btnEnter(BtnEnter), btnStop(BtnStop), 
    motors(Motors), log(Log), bt(Bt),
    speedStart(20),
    distanceTargetDiff(100),
    errorPosThreshold(80),
    rotateSensorThreshold(30),
    rotateSensorDistanceDiff(150),
    debugGlobal(DebugGlobal)
    {}

    void debugCheckGlobal(Debug& local) {
        if(local == Debug::Default)
            local = debugGlobal;
    }

    void init() {
        // Init ColorSensor:
        // colorL.reflected() is blocking function 
        // => If the sensor is not connected then the program freezes
        colorL.reflected();
        colorR.reflected();
    }

    void calibrateSensor(Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);
        const int robotCalibrationDegrees = 360;

        // motors.onForDegrees(-10, 10, robotCalibrationDegrees/2, false, true);
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        motors.onForDegrees(10, -10, 
            robotGeometry.distanceDegrees(robotGeometry.rotateDegrees(robotCalibrationDegrees)), true, false);
        
        while((motors.leftMotor().degrees() < (robotCalibrationDegrees - 10)) &&
              (motors.rightMotor().degrees() < (robotCalibrationDegrees - 10)))
        {
        //while(robotCalibrationDegrees == approximately(motors.leftMotor().degrees() , 20)) {
            ev3cxx::delayMs(10);

            colorL.calibrateReflection();
            colorR.calibrateReflection();
        }
        motors.off(false);
        // ev3cxx::delayMs(500);
        // motors.leftMotor().resetPosition();
        // motors.rightMotor().resetPosition();
        // motors.onForDegrees(-10, 10, robotCalibrationDegrees/2, false, true);
    }

    void packetColorReflected() {
        int lCalVal = colorL.reflected(true, false);
        int rCalVal = colorR.reflected(true, false);

        atoms::AvakarPacket packetOut;
        packetOut.set_command(0);
        packetOut.push<uint8_t>(lCalVal);
        packetOut.push<uint8_t>(rCalVal);
        packetOut.push<uint8_t>(lCalVal + rCalVal);
        packetOut.push<int8_t>(lCalVal - rCalVal);
        for(char ch: packetOut) {
            bt.write(ch); 
        }
        packetOut.clear();
    }

    void calculateLine(int& errorNeg, int& errorPos, 
                       Debug debugLocal = Debug::Default) {
        int lCalVal = colorL.reflected(true, false);
        int rCalVal = colorR.reflected(true, false);

        lAvgVal.push(lCalVal);
        rAvgVal.push(rCalVal);

        lCalVal = lAvgVal.get_average();
        rCalVal = rAvgVal.get_average();

        errorNeg = rCalVal - lCalVal;
        errorPos = rCalVal + lCalVal;
        
        // debugCheckGlobal(debugLocal);
        // if(debugLocal == Debug::Packet)
        //     packet_send_color_sensors(log, lCalVal, rCalVal, errorNeg);
    }

    State _step(Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);

        log.logInfo("_STEP", "_STEP");

        int distanceTarget = motors.leftMotor().degrees() + distanceTargetDiff;
        lAvgVal.clear();
        rAvgVal.clear();

        atoms::RollingAverage<int, 3> lAvgVal, rAvgVal;

        while(true) {
            int errorNeg, errorPos; 
            // calculateLine(errorNeg, errorPos, debugLocal);

            int lCalVal = colorL.reflected(true, false);
            int rCalVal = colorR.reflected(true, false);

            lAvgVal.push(lCalVal);
            rAvgVal.push(rCalVal);

            lCalVal = lAvgVal.get_average();
            rCalVal = rAvgVal.get_average();

            errorNeg = rCalVal - lCalVal;
            errorPos = rCalVal + lCalVal;

            int errorLine = errorNeg / 12;

            int motorLSpeed = speedStart + errorLine;
            int motorRSpeed = speedStart - errorLine;

            if(btnStop.isPressed()) {
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            } else if(touchStop.isPressed()) {
                motors.off();
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
            } else if(distanceTarget < motors.leftMotor().degrees() // if distance from last crossline is bigger then distanceTarget
                   && errorPos < errorPosThreshold) 
            {
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);
                return State::PositionReached;
            } else {
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::GREEN);
                motors.on(motorRSpeed, motorLSpeed);
            }

            // if(debugLocal == Debug::Packet)
            //     packet_send_motors_line(log, motorLSpeed, motorRSpeed, errorLine);

            // if(DETECTOR) {
            //     return State::RivalDetected;
            // }

            ev3cxx::delayMs(5);
        }
    }

    State step(int numberOfStep, Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);
        State returnState;

        log.logInfo("MOVE", "Step: {}") << numberOfStep;

        for(int cntOfStep = 0; cntOfStep < numberOfStep; ++cntOfStep) {
            returnState = _step(debugLocal);
            if(returnState != State::PositionReached)
                return returnState;
        }
        motors.onForDegrees(speedStart, speedStart, 
            robotGeometry.distanceDegrees(100));
        return returnState;
    }

    State stepBack(int numberOfStep, Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);
        
        motors.onForDegrees(-speedStart, -speedStart, 
            robotGeometry.distanceDegrees(200 * numberOfStep));
        return State::PositionReached;
        // TODO
    }

    // rotate() - int degrees => positive number rotate in mathematic direction (anticlock wise)
    void rotateOdometer(int degrees, Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);
        
        motors.onForDegrees(speedStart, -speedStart, 
            robotGeometry.distanceDegrees(robotGeometry.rotateDegrees(degrees)));
    }

    //State rotateSensor(bool clockWise = true, Debug debugLocal = Debug::Default) {
    //State rotateSensor(Direction dir = Direction::Left, Debug debugLocal = Debug::Default) {

    State rotate(int degrees, Debug debugLocal = Debug::Default) {
        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();

        log.logInfo("MOVE", "Rotate: {}") << degrees;
        
        if(degrees > 0) {
            motors.on(speedStart, -speedStart);
        } else {
            motors.on(-speedStart, speedStart);
        }

        while(motors.leftMotor().degrees() < rotateSensorDistanceDiff && 
              motors.rightMotor().degrees() < rotateSensorDistanceDiff) 
        {
            if(btnStop.isPressed()) {
                motors.off(false);
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            }
        }

        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
        while(colorL.reflected() > rotateSensorThreshold &&
              colorR.reflected() > rotateSensorThreshold) 
        {
           if(btnStop.isPressed()) {
                motors.off(false);
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            }
            ev3cxx::delayMs(50);
        }

        motors.off();
        return State::PositionReached;
    }

    State rotateReflected(int degrees, Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);

        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::GREEN);

        int rotateSensorDistance = 
            motors.leftMotor().degrees() + rotateSensorDistanceDiff;

        Direction dir = Direction::Left;
        if(degrees < 0) {
            dir = Direction::Right;
        }

        motors.on(int(dir) * speedStart, int(dir) * -speedStart);
        
        // if(dir == Direction::Left)
        //     ev3cxx::ColorSensor& colorSenRotate = colorL;
        // else
        //     ev3cxx::ColorSensor& colorSenRotate = colorR;
        
        while(true) {
        //     if(debugLocal == Debug::Packet)
        //         packetColorReflected();
            if(btnStop.isPressed()) {
                motors.off(false);
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            }
            // else if(rotateSensorThreshold < motors.leftMotor().degrees() // if distance from last crossline is bigger then distanceTarget
            //      && errorPos < errorPosThreshold) {
            // }
            if(dir == Direction::Left) {
                if(rotateSensorDistance < motors.rightMotor().degrees()) {
                    if(colorL.reflected() < rotateSensorThreshold) {
                        motors.off(true);
                         return State::PositionReached;
                    }
                }
            } else {
                if(rotateSensorDistance < motors.leftMotor().degrees()) {
                    if(colorR.reflected() < rotateSensorThreshold) {
                        motors.off(true);
                        return State::PositionReached;
                    }
                }
            }
            ev3cxx::delayMs(5);
        }
    }

    Debug debugState() {
        return debugGlobal;
    }

    RobotGeometry& robotGeometry;
    ev3cxx::ColorSensor& colorL;
    ev3cxx::ColorSensor& colorR;
    ev3cxx::TouchSensor& touchStop;
    ev3cxx::BrickButton& btnEnter;
    ev3cxx::BrickButton& btnStop;

    ev3cxx::MotorTank& motors;

    Logger& log;
    ev3cxx::Bluetooth& bt;

    int speedStart;
    int distanceTargetDiff;
    int errorPosThreshold;
    int rotateSensorThreshold;
    int rotateSensorDistanceDiff;
    Debug debugGlobal;

private:
    atoms::RollingAverage<int, 3> lAvgVal, rAvgVal;
};
