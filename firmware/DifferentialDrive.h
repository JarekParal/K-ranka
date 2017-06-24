
#pragma once  

#include "ev3cxx.h"
#include "RobotGeometry.h"

class DifferentialDrive : public RobotGeometry {
public:
	typedef float wheelSizeType;

	DifferentialDrive(wheelSizeType wheelDiameter, wheelSizeType wheelBase, ev3cxx::MotorTank& motors)
	: RobotGeometry(wheelDiameter, wheelBase), m_motors(motors)  
	{}

	void onForMillimeters(int speed, int mm, bool brake = true, bool blocking = true, unsigned int wait_after_ms = 60) {
		m_motors.onForDegrees(speed, speed, distanceToDegrees(mm), brake, blocking, wait_after_ms);
	}

	void rotateOnDegrees(int speed, int degrees, bool brake = true, bool blocking = true, unsigned int wait_after_ms = 60) {
        m_motors.onForDegrees(speed, -speed, rotateDegrees(degrees), brake, blocking, wait_after_ms);
	}

private:
	ev3cxx::MotorTank& m_motors;
};
