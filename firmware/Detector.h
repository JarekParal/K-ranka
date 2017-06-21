
template < typename T >
struct Approximately {
    T _bottom, _up;
};

template < typename T >
bool operator==( T val, Approximately< T > ap ) {
    return val > ap._bottom && val < ap._up;
}

template < typename T >
Approximately< T > approximately( T val, T tolerance ) {
    return { val - tolerance, val + tolerance };
}

enum State {
    Clear = 0,
    Enemy,
    Unsure,
    Messure
}


class Detector
{
    public:
        Detector(SensorPort Top, SensorPort Side, int trashold, int collision){
            this.trashold = trashold;
            this.collision = collision;
            top = UltrasonicSensor{Top};
            side = UltrasonicSensor{Side};
        }

        State detect()
        {
            int top = topSensor.centimeters();
            int side = topSensor.centimeters();

            if (top > collision && side > collision){
                return State.Clear;
            }

            if (top < collision && side < collision){
                return State.Enemy;
            }

            lastDistanceTop = top;
            lastDistanceSide = side;

            return State.Unsure;
        }

        State reDetect()
        {
            if (lastDistanceTop == -1){
                return State.Messure;
            }
            
            if (top > collision && side > collision){
                return State.Clear;
            }

            if (top < collision && side < collision){
                return State.Enemy;
            }

            int top = topSensor.centimeters();
            int side = topSensor.centimeters();

            if (lastDistanceTop != approximately(top, trashold) || lastDistanceSide != approximately(side, trashold)){
                lastDistanceTop = top;
                lastDistanceSide = side;

                return State.Enemy;
            }

            return State.Clear;
        }

    private:
        int lastDistanceTop = -1;
        int lastDistanceSide = -1;

        int collision = 0;
        int trashold = 0;

        ev3cxx::UltrasonicSensor topSensor;
        ev3cxx::UltrasonicSensor sideSensor;
}
