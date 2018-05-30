#pragma once

#include <set>
#include <iostream>
#include "bfgrid.hpp"
#include <libs/logging/logging.hpp>

extern Logger l;

struct KetchupLogic {
    KetchupLogic( Robot& r )
        : position( 0, 3, Pred::East ),
          opponentValidFor( 0 ),
          ketchupCount( 0 ),
          lastUnloadPosition( 3 ),
          robot( r ) {}

    void go( const Position& p ) {
        while( true ) {
            std::set< Position > f = occupied;
            if ( opponentValidFor ) {
                f.insert( opponent );
                opponentValidFor--;
            }
            auto pathMap = shortestPaths( position, { 7, 7 }, f );
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
            if ( p == static_cast< Position >( position ) )
                return;
        }
    }

    Robot::State step() {
        switch( position.orient ) {
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
            assert( false );
        }
        return robot.step( 1 );
    }

    void onKetchup() {
        ketchupCount++;
        if ( ketchupCount == 1 )
            unload();
    }

    void unload() {
        l.logInfo( "", "Unloading" );
        if ( lastUnloadPosition == 4 ) {
            l.logInfo( "", "All occupied" );
            return; // Do not unload
        }
        l.logInfo( "", "Going to unload" );
        Position origin = position;
        go( { 0, lastUnloadPosition } );
        face( Pred::North );
        robot.openGate();
        go( { 0, lastUnloadPosition + 2 } );
        occupied.insert( { 0, lastUnloadPosition } );
        lastUnloadPosition++;
        ketchupCount = 0;
        occupied.insert( { 0, lastUnloadPosition } );
        robot.closeGate();
        l.logInfo( "", "Going back to work" );
        go( origin );
    }

    void onOpponent() {
        opponent = position;
        opponentValidFor = 4;
        face( invert( position.orient) );
        step();
    }

    void face( const Pred p ) {
        if ( position.orient == p )
            return;
        int rot = static_cast< int >( p ) - static_cast< int >( position.orient );
        if ( rot > 2 )
            rot -= 4;
        else if ( rot < -2 )
            rot += 4;
        robot.rotate( rot * 90 );
        position.orient = p;
    }

    RobotPosition position;

    Position opponent;
    int opponentValidFor;

    int ketchupCount;
    int lastUnloadPosition;

    std::set< Position > ketchups;

    std::set< Position > occupied;
    Robot& robot;
};