
#pragma once

#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h>

#include "ev3cxx.h"

#include "RobotGeometry.h"
#include "DifferentialDrive.h"

using ev3cxx::display;

struct LineSensor {
    LineSensor( ev3cxx::ColorSensor& s ) : _sensor( s ) { }

    int _val() {
        int r = _sensor.reflected( true, false );
        _aFast.push( r );
        _aSlow.push( r );
        return r;
    }

    int reflectedFast() {
        _val();
        return _aFast.get_average();
    }

    int reflectedSlow() {
        _val();
        return _aSlow.get_average();
    }

    ev3cxx::ColorSensor& _sensor;
    atoms::RollingAverage< int, 3 > _aFast;
    atoms::RollingAverage< int, 20 > _aSlow;
};

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
        StopBtnPressed,
        KetchupDetected
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
    lineL(ColorL), lineR(ColorR), ketchupSensor(TouchStop), btnEnter(BtnEnter), btnStop(BtnStop),
    motors(Motors), log(Log), bt(Bt),
    forwardSpeed(20),
    distanceTargetDiff(100),
    errorPosThreshold(100),
    rotateSensorThreshold(30),
    rotateSensorDistanceDiff(170),
    debugGlobal(DebugGlobal)
    {}

    void debugCheckGlobal(Debug& local) {
        if(local == Debug::Default)
            local = debugGlobal;
    }

    void init() {
        // Init ColorSensor:r
        // colorL.reflected() is blocking function
        // => If the sensor is not connected then the program freezes
        lineL._sensor.reflected();
        lineR._sensor.reflected();
    }

    void calibrateSensor(Debug debugLocal = Debug::Default) {
        debugCheckGlobal(debugLocal);
        const int robotCalibrationDegrees = 360;

        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        motors.onForDegrees(10, -10,
            robotGeometry.distanceDegrees(
                robotGeometry.rotateDegrees(robotCalibrationDegrees)), true, false);

        while((motors.leftMotor().degrees() < (robotCalibrationDegrees - 10)) &&
              (motors.rightMotor().degrees() < (robotCalibrationDegrees - 10)))
        {
            ev3cxx::delayMs(10);

            lineL._sensor.calibrateReflection();
            lineR._sensor.calibrateReflection();
        }
        motors.off(false);
    }

    std::pair< int, int > lineError() {
        int l = lineL.reflectedFast();
        int r = lineR.reflectedFast();;

        return { r - l, r + l };
    }

    State _step(Debug debugLocal = Debug::Default) {
        int target = motors.leftMotor().degrees() + distanceTargetDiff;
        lineR._aSlow.clear( 100 );
        while( true ) {
            int errorNeg = lineR.reflectedFast() - lineL.reflectedFast();
            int errorPos = lineR.reflectedSlow() + lineL.reflectedSlow();

            int speedGain = errorNeg / 12;
            int motorLSpeed = forwardSpeed + speedGain;
            int motorRSpeed = forwardSpeed - speedGain;

            if( btnStop.isPressed() ) {
                ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
                return State::StopBtnPressed;
            }
            else if ( motors.leftMotor().degrees() > target && errorPos < errorPosThreshold )
            {
                ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::ORANGE );
                return State::PositionReached;
            }
            else if ( !ketchupSensor.isPressed() ) {
                motors.leftMotor().resetPosition();
                motors.rightMotor().resetPosition();
                while ( motors.leftMotor().degrees() < robotGeometry.distanceDegrees(70)) {
                    if( btnStop.isPressed() ) {
                        ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
                        return State::StopBtnPressed;
                    }
                }
                return State::KetchupDetected;
            }
            else {
                ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );
                motors.on( motorRSpeed, motorLSpeed );
            }

            // if(DETECTOR) {
            //     return State::RivalDetected;
            // }

            ev3cxx::delayMs(5);
        }
    }

    State step( int numberOfStep, Debug debugLocal = Debug::Default ) {
        State returnState;

        for ( int cntOfStep = 0; cntOfStep < numberOfStep; ++cntOfStep ) {
            returnState = _step( debugLocal );
            if ( returnState != State::PositionReached && returnState != State::KetchupDetected )
                return returnState;
        }
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        motors.on( forwardSpeed, forwardSpeed );
        while ( motors.leftMotor().degrees() < robotGeometry.distanceDegrees(70)) {
            if( btnStop.isPressed() ) {
                ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
                return State::StopBtnPressed;
            }
        }
        return returnState;
    }

    State rotate(int degrees, Debug debugLocal = Debug::Default) {
        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);
        lineR._aSlow.clear( 100 );
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();

        float steps = 1.10 * abs( degrees ) / 90;

        if(degrees > 0) {
            motors.on(forwardSpeed, -forwardSpeed);
        } else {
            motors.on(-forwardSpeed, forwardSpeed);
        }

        while(motors.leftMotor().degrees() < steps * rotateSensorDistanceDiff &&
              motors.rightMotor().degrees() < steps * rotateSensorDistanceDiff)
        {
            if(btnStop.isPressed()) {
                motors.off(false);
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            }
            ev3cxx::delayMs(1);
        }

        ev3cxx::delayMs(1);

        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
        while(lineL.reflectedSlow() > rotateSensorThreshold &&
              lineR.reflectedSlow() > rotateSensorThreshold)
        {
           if(btnStop.isPressed()) {
                motors.off(false);
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            }
            ev3cxx::delayMs(1);
        }
        log.logInfo("", "X: {}, {}", lineL.reflectedSlow(), lineR.reflectedSlow() );

        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        while( motors.leftMotor().degrees() < 10 &&
               motors.rightMotor().degrees() < 10)
        {
            if(btnStop.isPressed()) {
                motors.off(false);
                ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);
                return State::StopBtnPressed;
            }
            ev3cxx::delayMs(1);
        }

        motors.off();
        return State::PositionReached;
    }

    Debug debugState() {
        return debugGlobal;
    }

    RobotGeometry& robotGeometry;
    LineSensor lineL;
    LineSensor lineR;
    ev3cxx::TouchSensor& ketchupSensor;
    ev3cxx::BrickButton& btnEnter;
    ev3cxx::BrickButton& btnStop;

    ev3cxx::MotorTank& motors;

    Logger& log;
    ev3cxx::Bluetooth& bt;

    int forwardSpeed;
    int distanceTargetDiff;
    int errorPosThreshold;
    int rotateSensorThreshold;
    int rotateSensorDistanceDiff;
    Debug debugGlobal;
};
