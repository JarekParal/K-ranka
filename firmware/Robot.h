
#pragma once

#define ATOMS_NO_EXCEPTION

#include <atoms/communication/avakar.h>
#include <atoms/numeric/rolling_average.h>

#include "ev3cxx.h"

#include "RobotGeometry.h"
#include "DifferentialDrive.h"

using ev3cxx::display;


template < typename T >
int sgn( T val )
{
	return ( T( 0 ) < val ) - ( val < T( 0 ) );
}


struct LineSensor
{
	LineSensor( ev3cxx::ColorSensor& s ) : _sensor( s ) { }


	int _val( )
	{
		int r = _sensor.reflected( true, false );
		_aFast.push( r );
		_aSlow.push( r );
		return r;
	}


	int reflectedFast( )
	{
		_val();
		return _aFast.get_average();
	}


	int reflectedSlow( )
	{
		_val();
		return _aSlow.get_average();
	}


	ev3cxx::ColorSensor& _sensor;
	atoms::RollingAverage < int, 3 > _aFast;
	atoms::RollingAverage < int, 20 > _aSlow;
};


void packet_send_color_sensors( ev3cxx::Bluetooth& bt, int lCalVal, int rCalVal, int errorNeg )
{
	atoms::AvakarPacket packetOut;

	packetOut.set_command( 0 );
	packetOut.push < uint8_t >( lCalVal );
	packetOut.push < uint8_t >( rCalVal );
	packetOut.push < uint8_t >( lCalVal + rCalVal );
	packetOut.push < int8_t >( errorNeg );
	for ( char ch: packetOut ) {
		bt.write( ch );
	}
	packetOut.clear();
}


void packet_send_motors_line( ev3cxx::Bluetooth& bt, int motorLSpeed, int motorRSpeed, int errorNegLine )
{
	atoms::AvakarPacket packetOut;

	packetOut.set_command( 1 );
	packetOut.push < int16_t >( motorLSpeed );
	packetOut.push < int16_t >( motorRSpeed );
	packetOut.push < int16_t >( errorNegLine );
	for ( char ch: packetOut ) {
		bt.write( ch );
	}
	packetOut.clear();
}


void beepVolume( int vol )
{
	ev3_speaker_set_volume( vol );
}


void beep( unsigned frequency, int durationMs = 500 )
{
	ev3_speaker_play_tone( frequency, durationMs );
}


void beepTest( )
{
	for ( int i = 0; i <= 5; ++i ) {
		beepVolume( 20 * i );
		beep( 100, 500 );
		ev3cxx::delayMs( 500 );
	}

	for ( int frec = 1; frec < 30; ++frec ) {
		for ( int i = 0; i <= 5; ++i ) {
			beepVolume( 20 * i );
			beep( 500 * frec, 500 );
			ev3cxx::delayMs( 500 );
		}
	}
}


class Robot
{
public:
	enum class State
	{
		PositionReached = 0,
		KetchupDetected,
		RivalDetected,
	};

	// static std::string const StateStr[3] = {
	//     "PositionReached",
	//     "KetchupDetected",
	//     "RivalDetected"
	// };

	enum Debug
	{
		No = 1 << 0,
		Text = 1 << 1,
		Packet = 1 << 2,
		Default = 1 << 3,
		TextPacket = Text | Packet
	};

	enum class Direction
	{
		Left = -1,
		Right = 1
	};


	Robot( RobotGeometry& rGeometry, ev3cxx::ColorSensor& ColorL, ev3cxx::ColorSensor& ColorR,
	       ev3cxx::TouchSensor& TouchStop,
	       ev3cxx::BrickButton& BtnEnter, ev3cxx::BrickButton& BtnStop, ev3cxx::MotorTank& Motors,
	       ev3cxx::Motor& MotorGate,
	       Logger& Log, ev3cxx::Bluetooth& Bt, ev3cxx::UltrasonicSensor& sonar, ev3cxx::Motor& motorSensor,
	       Debug DebugGlobal = Debug::No )
			: robotGeometry( rGeometry ),
			  lineL( ColorL ), lineR( ColorR ), ketchupSensor( TouchStop ), btnEnter( BtnEnter ), btnStop( BtnStop ),
			  motors( Motors ), motorGate( MotorGate ), log( Log ), bt( Bt ),
			  forwardSpeed( 20 ),
			  distanceTargetDiff( 100 ),
			  errorPosThreshold( 100 ),
			  rotateSensorThreshold( 30 ),
//			  rotateSensorThreshold( 10 ),
			  rotateSensorDistanceDiff( 170 ),
			  debugGlobal( DebugGlobal ),
			  sonar( sonar ),
			  motorSensor( motorSensor ) { }


	void debugCheckGlobal( Debug& local )
	{
		if ( local == Debug::Default )
			local = debugGlobal;
	}


	void exit( int exitCode )
	{
		log.logWarning( "EXIT", "exit()" );
		ledRed();
		motorsBrake();
		ev3cxx::delayMs( 500 );
		std::exit( exitCode );
	}


	void motorsBrake( bool brake = false )
	{
		motors.off( brake );
		motorGate.off( brake );
	}


	void init( )
	{
		// Init ColorSensor:r
		// colorL.reflected() is blocking function
		// => If the sensor is not connected then the program freezes
		lineL._sensor.reflected();
		lineR._sensor.reflected();
		gateInit();
		brakeGate();

		closeSensorArm();
		brakeSensorArm();
	}


	void ledOff( )
	{
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::OFF );
	}


	void ledRed( )
	{
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
	}


	void ledGreen( )
	{
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );
	}


	void ledOrange( )
	{
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::ORANGE );
	}


	void gateInit( )
	{
		closeGate();
	}


	void brakeGate( )
	{
		motorGate.off( true );
	}


	void openGate( )
	{
		// const float gatePositionOpen = 0.26;
		// motorGate.resetPosition();
		// motorGate.onForRotations(-20, gatePositionOpen, true, false);
		motorGate.on( 20 );
		ev3cxx::delayMs( 1000 );
		brakeGate();
	}


	void closeGate( )
	{
		// const float gatePositionClose = 0.26;
		// motorGate.resetPosition();
		// motorGate.onForRotations(20, gatePositionClose, true, false);
		motorGate.on( -20 );
		ev3cxx::delayMs( 1000 );
		brakeGate();
	}


	void brakeSensorArm( )
	{
		motorSensor.off( true );
	}


	void openSensorArm( )
	{
		// const float gatePositionOpen = 0.26;
		// motorGate.resetPosition();
		// motorGate.onForRotations(-20, gatePositionOpen, true, false);
		motorSensor.on( 20 );
		ev3cxx::delayMs( 700 );
		brakeSensorArm();
	}


	void closeSensorArm( )
	{
		// const float gatePositionClose = 0.26;
		// motorGate.resetPosition();
		// motorGate.onForRotations(20, gatePositionClose, true, false);
		motorSensor.on( -20 );
		ev3cxx::delayMs( 700 );
		brakeSensorArm();
	}


	void calibrateSensor( Debug debugLocal = Debug::Default )
	{
		debugCheckGlobal( debugLocal );
		const int robotCalDeg = 360;

		motors.leftMotor().resetPosition();
		motors.rightMotor().resetPosition();
		motors.on( 25, -25 );
		while ( ( motors.leftMotor().degrees() < robotGeometry.rotateDegrees( robotCalDeg ) ) &&
		        ( motors.rightMotor().degrees() < robotGeometry.rotateDegrees( robotCalDeg ) ) ) {
			ev3cxx::delayMs( 10 );

			lineL._sensor.calibrateReflection();
			lineR._sensor.calibrateReflection();
		}
		motors.off( false );
	}



	bool enemyDetected( )
	{
		return false;
		return sonar.centimeters() < 20;
	}


	std::pair < int, int > lineError( )
	{
		int l = lineL.reflectedFast();
		int r = lineR.reflectedFast();;

		return { r - l, r + l };
	}


	State _moveForward( int distanceMm )
	{
		int distanceDeg = robotGeometry.distanceToDegrees( distanceMm );
		motors.leftMotor().resetPosition();
		motors.rightMotor().resetPosition();
		motors.on( forwardSpeed, forwardSpeed );
		ev3cxx::delayMs( 5 );
		while ( motors.leftMotor().degrees() < distanceDeg &&
		        motors.rightMotor().degrees() < distanceDeg ) {
			// log.logInfo("DEBUG", "R: {} - {}") << distanceDeg << motors.rightMotor().degrees();

			if ( btnStop.isPressed() )
				exit( 1 );
			if ( enemyDetected() )
				return State::RivalDetected;
			ev3cxx::delayMs( 5 );
		}

		// log.logInfo("DEBUG", "R: {} - {}") << position;
		return State::PositionReached;
	}


	State _moveBackward( int distanceMm )
	{
		int distanceDeg = -robotGeometry.distanceToDegrees( distanceMm );
		motors.leftMotor().resetPosition();
		motors.rightMotor().resetPosition();
		motors.on( -forwardSpeed, -forwardSpeed );
		ev3cxx::delayMs( 5 );
		while ( motors.leftMotor().degrees() > distanceDeg &&
		        motors.rightMotor().degrees() > distanceDeg ) {
			// log.logInfo("DEBUG", "R: {} - {}") << distanceDeg << motors.rightMotor().degrees();

			if ( btnStop.isPressed() )
				exit( 1 );
			if ( enemyDetected() )
				return State::RivalDetected;
			ev3cxx::delayMs( 5 );
		}
		return State::PositionReached;
	}


	void  findLine( ){
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );
		lineL._aSlow.clear( 100 );
		lineR._aSlow.clear( 100 );

		int degrees = 40;
		_rotate( degrees );

		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::ORANGE );

//		if ( degrees > 0 ) {
		motors.on( -10, 10 );
//		} else {
//			motors.on( 10, -10 );
//		}

		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
		while ( lineL.reflectedSlow() > rotateSensorThreshold &&
		        lineR.reflectedSlow() > rotateSensorThreshold ) {
			if ( btnStop.isPressed() )
				exit( 1 );

			ev3cxx::delayMs( 1 );
		}
//
		while ( lineL.reflectedSlow() < rotateSensorThreshold ||
		        lineR.reflectedSlow() < rotateSensorThreshold ) {
			if ( btnStop.isPressed() )
				exit( 1 );

			ev3cxx::delayMs( 1 );
		}

		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );

		motors.off();
	}

	State _rotate( int degrees )
	{
		int distanceDeg = robotGeometry.rotateDegrees( degrees );
		motors.leftMotor().resetPosition();
		motors.rightMotor().resetPosition();
		if ( degrees > 0 )
			motors.on( forwardSpeed, -forwardSpeed );
		else
			motors.on( -forwardSpeed, forwardSpeed );

		while ( motors.leftMotor().degrees() < distanceDeg &&
		        motors.rightMotor().degrees() < distanceDeg ) {
			if ( btnStop.isPressed() ) {
				exit( 1 );
			}
			ev3cxx::delayMs( 5 );
		}
		return State::PositionReached;
	}


	State _step( Debug debugLocal = Debug::Default )
	{
		int target = robotGeometry.distanceToDegrees( 40 );
		// int target = robotGeometry.distanceToDegrees( 80 );
		motors.leftMotor().resetPosition();
		motors.rightMotor().resetPosition();
		lineL._aSlow.clear( 100 );
		lineR._aSlow.clear( 100 );

		atoms::RollingAverage < float, 50 > ketchupTrigger;
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
		while ( true ) {
			int errorNeg = lineR.reflectedFast() - lineL.reflectedFast();
			int errorPos = lineR.reflectedSlow() + lineL.reflectedSlow();

			int speedGain = errorNeg / 12;
			// int motorLSpeed = forwardSpeed + speedGain;
			// int motorRSpeed = forwardSpeed - speedGain;

			int motorLSpeed = forwardSpeed - speedGain;
			int motorRSpeed = forwardSpeed + speedGain;

			// log
			// log.logInfo("DEBUG", "S: {} - {}") << motors.rightMotor().degrees() << target;
			// log.logInfo("DEBUG", "G: {} - {}") << motorLSpeed << motorRSpeed;

			if ( btnStop.isPressed() ) {
				exit( 1 );
			}
			if ( motors.rightMotor().degrees() > target && errorPos < errorPosThreshold ) {

				motors.on( motorRSpeed, motorLSpeed );
				// ev3cxx::delayMs(25);
				// motors.off();

				ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::ORANGE );
				beep( 1000, 200 );
				return State::PositionReached;
			}
			// if ( !ketchupSensor.isPressed() ) {
			if ( ketchupSensor.isPressed() ) {
				ketchupTrigger.push( 1 );
			} else {
				ketchupTrigger.push( 0 );
			}
			// if ( ketchupTrigger.get_average() > 0.9 ) {
			if ( ketchupTrigger.get_average() > 0.2 ) {
				motors.off();
				ev3cxx::delayMs( 100 );
				openSensorArm();
				ev3cxx::delayMs( 100 );
				auto s = _moveForward( 110 );
				ev3cxx::delayMs( 5 );
				closeSensorArm();
				s = _moveBackward( 40 );
				motors.off();
				ev3cxx::delayMs( 1000 );

				findLine();
				beep( 2000, 200 );
				ev3cxx::delayMs( 1000 );
				return std::max( State::KetchupDetected, s );
			}
			if ( enemyDetected() ) {
				// beep( 400, 200 );
				// return State::RivalDetected;
			}
			motors.on( motorRSpeed, motorLSpeed );
			ev3cxx::delayMs( 5 );
		}
	}


	State step( int numberOfStep, Debug debugLocal = Debug::Default )
	{
		State returnState;

		for ( int cntOfStep = 0; cntOfStep < numberOfStep; ++cntOfStep ) {
			returnState = _step( debugLocal );
			if ( returnState != State::PositionReached && returnState != State::KetchupDetected ) {
				return returnState;
			}
		}
		// auto s = _moveForward( 70 );
		// 85 relativně fungovalo

		State s;
		if (returnState != State::KetchupDetected) {
			s = _moveForward( 100 );
		}

		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
		return std::max( returnState, s );
	}

	//Dotáčet se podle čáry
	State rotate( int degrees, Debug debugLocal = Debug::Default )
	{
		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );
		lineL._aSlow.clear( 100 );
		lineR._aSlow.clear( 100 );

		int odoDeg = sgn( degrees ) * ( abs( degrees ) - 40 );
		_rotate( odoDeg );

//		LineSensor* crossS = nullptr;
		if ( degrees > 0 ) {
			motors.on( 10, -10 );
		} else {
			motors.on( -10, 10 );
		}


		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::RED );
		while ( lineL.reflectedSlow() > rotateSensorThreshold &&
		        lineR.reflectedSlow() > rotateSensorThreshold ) {
			if ( btnStop.isPressed() )
				exit( 1 );

			ev3cxx::delayMs( 1 );
		}

		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::ORANGE );

		if ( degrees > 0 ) {
			motors.on( 10, -10 );
		} else {
			motors.on( -10, 10 );
		}

		while ( lineL.reflectedSlow() < rotateSensorThreshold ||
		        lineR.reflectedSlow() < rotateSensorThreshold ) {
			if ( btnStop.isPressed() )
				exit( 1 );

			ev3cxx::delayMs( 1 );
		}

		ev3cxx::statusLight.setColor( ev3cxx::StatusLightColor::GREEN );
//		_rotate( sgn( degrees ) * 10 );

		motors.off();
		return State::PositionReached;
	}


	Debug debugState( )
	{
		return debugGlobal;
	}


	RobotGeometry& robotGeometry;
	LineSensor lineL;
	LineSensor lineR;
	ev3cxx::TouchSensor& ketchupSensor;
	ev3cxx::BrickButton& btnEnter;
	ev3cxx::BrickButton& btnStop;

	ev3cxx::MotorTank& motors;
	ev3cxx::Motor& motorGate;
	ev3cxx::Motor& motorSensor;

	Logger& log;
	ev3cxx::Bluetooth& bt;

	int forwardSpeed;
	int distanceTargetDiff;
	int errorPosThreshold;
	int rotateSensorThreshold;
	int rotateSensorDistanceDiff;
	Debug debugGlobal;

	ev3cxx::UltrasonicSensor& sonar;
};
