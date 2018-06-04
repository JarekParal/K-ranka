
#pragma once

template < typename T >
struct Approximately {
    T _bottom, _up;
};

template < typename T >
bool operator==( T val, Approximately< T > ap ) {
    return val > ap._bottom && val < ap._up;
}

template < typename T >
bool operator!=( T val, Approximately< T > ap ) {
    return val < ap._bottom || val > ap._up;
}

template < typename T >
Approximately< T > approximately( T val, T tolerance ) {
    return { val - tolerance, val + tolerance };
}

#include "ev3cxx.h"

class Detector
{
    public:
        Detector(ev3cxx::UltrasonicSensor& topSensor, ev3cxx::UltrasonicSensor& sideSensor, int trashold, int collision): topSensor(topSensor), sideSensor(sideSensor){
            this->trashold = trashold;
            this->collision = collision;
        }

        enum State {
            Clear = 0,
            Enemy,
            Unsure,
            Messure
        };

        State detect()
        {
            int top = topSensor.centimeters();
            int side = topSensor.centimeters();

            if (top > collision && side > collision){
                return State::Clear;
            }

            if (top < collision && side < collision){
                return State::Enemy;
            }

            lastDistanceTop = top;
            lastDistanceSide = side;

            return State::Unsure;
        }

        State reDetect()
        {
            if (lastDistanceTop == -1){
                return State::Messure;
            }

            int top = topSensor.centimeters();
            int side = topSensor.centimeters();
            
            return State::Clear;

            if (top > collision && side > collision){
                return State::Clear;
            }

            if (top < collision && side < collision){
                return State::Enemy;
            }

            if (lastDistanceTop != approximately(top, trashold) || lastDistanceSide != approximately(side, trashold)){
                lastDistanceTop = top;
                lastDistanceSide = side;

                return State::Enemy;
            }

            return State::Clear;
        }

    private:
        int lastDistanceTop = -1;
        int lastDistanceSide = -1;

        int collision = 0;
        int trashold = 0;

        ev3cxx::UltrasonicSensor topSensor;
        ev3cxx::UltrasonicSensor sideSensor;
};
