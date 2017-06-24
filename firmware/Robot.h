
#pragma once

#define ATOMS_NO_EXCEPTION
#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h>

#include "ev3cxx.h"

#include "RobotGeometry.h"
#include "DifferentialDrive.h"

using ev3cxx::display;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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
        PositionReached = 0,
        KetchupDetected,
        RivalDetected,
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
        const int robotCalDeg = 360;

        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        motors.on(25, -25);
        while((motors.leftMotor().degrees() < robotGeometry.rotateDegrees(robotCalDeg)) &&
              (motors.rightMotor().degrees() < robotGeometry.rotateDegrees(robotCalDeg)))
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

    State _moveForward( int distanceMm ) {
        int distanceDeg = robotGeometry.distanceToDegrees( distanceMm );
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        motors.on( forwardSpeed, forwardSpeed );
        ev3cxx::delayMs( 5 );
        while( motors.leftMotor().degrees() < distanceDeg &&
               motors.rightMotor().degrees() < distanceDeg )
        {
            if ( btnStop.isPressed() ) {
                std::exit( 1 );
            }
            ev3cxx::delayMs( 5 );
        }
        return State::PositionReached;
    }

    State _rotate( int degrees ) {
        int distanceDeg = robotGeometry.rotateDegrees( degrees );
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        if ( degrees > 0)
            motors.on( forwardSpeed, -forwardSpeed );
        else
            motors.on( -forwardSpeed, forwardSpeed );

        while( motors.leftMotor().degrees() < distanceDeg &&
               motors.rightMotor().degrees() < distanceDeg )
        {
            if ( btnStop.isPressed() ) {
                std::exit( 1 );
            }
             ev3cxx::delayMs(5);
        }
        return State::PositionReached;
    }

    State _step(Debug debugLocal = Debug::Default) {
        int target = robotGeometry.distanceToDegrees( 40 );
        motors.leftMotor().resetPosition();
        motors.rightMotor().resetPosition();
        lineL._aSlow.clear( 100 );
        lineR._aSlow.clear( 100 );

        atoms::RollingAverage< float, 50 > ketchupTrigger;
        ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );
        while( true ) {
            int errorNeg = lineR.reflectedFast() - lineL.reflectedFast();
            int errorPos = lineR.reflectedSlow() + lineL.reflectedSlow();

            int speedGain = errorNeg / 12;
            int motorLSpeed = forwardSpeed + speedGain;
            int motorRSpeed = forwardSpeed - speedGain;

            if ( btnStop.isPressed() ) {
                std::exit( 1 );
            }
            if ( motors.leftMotor().degrees() > target && errorPos < errorPosThreshold ) {
                ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::ORANGE );
                return State::PositionReached;
            }
            if ( !ketchupSensor.isPressed() ) {
                ketchupTrigger.push( 1 );
            }
            else {
                ketchupTrigger.push( 0 );
            }
            if ( ketchupTrigger.get_average() > 0.9 ) {
                auto s = _moveForward( 25 );
                return std::max( State::KetchupDetected, s );
            }

            motors.on( motorRSpeed, motorLSpeed );
            ev3cxx::delayMs(5);
        }
    }

    State step( int numberOfStep, Debug debugLocal = Debug::Default ) {
        State returnState;

        for ( int cntOfStep = 0; cntOfStep < numberOfStep; ++cntOfStep ) {
            returnState = _step( debugLocal );
            if ( returnState != State::PositionReached && returnState != State::KetchupDetected ) {
                return returnState;
            }
        }
        auto s = _moveForward( 70 );
        ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
        return std::max( returnState, s );
    }

    State rotate(int degrees, Debug debugLocal = Debug::Default) {
        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::GREEN);
        lineL._aSlow.clear( 100 );
        lineR._aSlow.clear( 100 );

        int odoDeg = sgn( degrees ) * ( abs( degrees ) - 40 );
        _rotate( odoDeg );

        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::RED);

        while(lineL.reflectedSlow() > rotateSensorThreshold &&
              lineR.reflectedSlow() > rotateSensorThreshold)
        {
           if ( btnStop.isPressed() )
                std::exit( 1 );
            ev3cxx::delayMs(1);
        }

        ev3cxx::statusLight.setColor(ev3cxx::StatusLightColor::ORANGE);
        _rotate( sgn( degrees ) * 10 );

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
