#pragma once

#include <set>
#include <iostream>
#include "bfgrid.hpp"
#include <libs/logging/logging.hpp>

extern Logger l;

struct KetchupLogic
{
	KetchupLogic( Robot& r )
			: position( 3, 0, Pred::North ),
//			: position( 1, 0, Pred::North ),
			  opponentValidFor( 0 ),
			  ketchupCount( 0 ),
			  lastUnloadPosition( 1 ),
//			  lastUnloadPosition( 3 ),
              robot( r ) { }


	void go( const Position& p )
	{
		while ( true ) {
			std::set < Position > f{ };

			for ( auto o: occupied ) {
				f.insert( o );
			}

			if ( opponentValidFor ) {
				f.insert( opponent );
				opponentValidFor--;
			}

			l.logInfo( "", "Going: {}, {}", position.x, position.y );
			auto pathMap = shortestPaths( position, { 7, 7 }, f );
//			auto pathMap = shortestPaths( position, { 3, 3 }, f );
			auto dir = pathMap.pathTo( p ).front();
			face( dir );
			auto status = step();
			l.logInfo( "", "Step done {}, {}", position.x, position.y );
			switch ( status ) {
				case Robot::State::RivalDetected:
					onOpponent();
					break;
				case Robot::State::KetchupDetected:
					onKetchup();
					break;
				default:
					break;
			}

			if ( p == static_cast< Position >( position ) ) {
				return;
			}
		}
	}


	Robot::State step( )
	{
		switch ( position.orient ) {
			case Pred::North:
				position.y++;
				break;
			case Pred::West:
				position.x--;
				break;
			case Pred::South:
				position.y--;
				break;
			case Pred::East:
				position.x++;
				break;
			default:
				l.logInfo( "", "Wrong orient" );
				ev3cxx::delayMs(1000);
				assert( false );
				break;
		}
		return robot.step( 1, Robot::Debug::Default, false, ketchupCount);
	}


	void onKetchup( )
	{
		ketchupCount++;
		if ( ketchupCount == 2 ) {
			unload();
		}
	}


	void unload( )
	{
		l.logInfo( "", "Unloading" );
		if ( lastUnloadPosition == 4 ) {
			l.logInfo( "", "All occupied" );
			return; // Do not unload
		}
		l.logInfo( "", "Going to unload" );
//		Position origin = position;
		int x = position.x;
		int y = position.y;

//		go({3, 0});
//		face( Pred::East );

//		robot.openSensorArm();
//		robot._moveBackward( 300 );
//		robot.exit( 1 );

		go( { lastUnloadPosition, 0 } );
		face( Pred::East );

		robot._moveForward(15);
		robot.motors.off();
		robot.openGate();
		go( { lastUnloadPosition + 2, 0 } );

		robot.motors.off();
		robot.closeGate();
		robot.findLine();
//		ev3cxx::delayMs( 1500 );

		occupied.insert( { lastUnloadPosition, 0 } );
		ketchupCount = 0;

		lastUnloadPosition++;
//		occupied.insert( { lastUnloadPosition, 0 } );



//		robot.findLine();

//		ev3cxx::delayMs( 500 );

//		robot._moveBackward( 50 );

		l.logInfo( "", "Back: [ {} ; {} ]" ) << x << y;

		ev3cxx::delayMs( 1500 );
//		go( origin );
		go( { x, y } );

//      go( { 0, lastUnloadPosition } );
//		robot.openGate();
//      go( { 0, lastUnloadPosition + 2 } );
//      occupied.insert( { 0, lastUnloadPosition } );
//      lastUnloadPosition++;
//      ketchupCount = 0;
//      occupied.insert( { 0, lastUnloadPosition } );
//      robot.closeGate();
//      l.logInfo( "", "Going back to work" );
//      go( origin );
	}


	void onOpponent( )
	{
		opponent = position;
		opponentValidFor = 4;
		face( invert( position.orient ) );
		step();
	}


	void face( const Pred p )
	{
		if ( position.orient == p ) {
			return;
		}

		int rot = static_cast< int >( p ) - static_cast< int >( position.orient );
		if ( rot > 2 ) {
			rot -= 4;
		} else if ( rot < -2 ) {
			rot += 4;
		}

		l.logInfo( "", "Rotation: [ {} ]" ) << rot;
		robot.rotate( rot * 90 );
		position.orient = p;
	}


	RobotPosition position;

	Position opponent;
	int opponentValidFor;

	int ketchupCount;
	int lastUnloadPosition;

	std::set < Position > ketchups;

	std::set < Position > occupied;
//	std::vector < Position > occupied;
	Robot& robot;
};