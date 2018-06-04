
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

class RobotGeometry {
public:
	typedef float wheelSizeType;

	RobotGeometry(wheelSizeType wheelDiameter, wheelSizeType wheelBase)
	: m_wheelDiameter(wheelDiameter), m_wheelBase(wheelBase) 
	{}

	float distanceToDegrees(float distance) {
		return (distance / (m_wheelDiameter * M_PI)) * 360; 	
	}

	float rotateDegrees(float degrees) {
		return distanceToDegrees(((m_wheelBase * M_PI) * degrees) / 360);
	}

	wheelSizeType wheelDiameter() {
		return m_wheelDiameter;
	}
	
	wheelSizeType wheelBase() {
		return m_wheelBase;
	}

private:
	wheelSizeType m_wheelDiameter;
	wheelSizeType m_wheelBase;
};
