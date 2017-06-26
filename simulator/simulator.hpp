#pragma once

#include <iostream>
#include <string>
#include <thread>
#include <map>
#include <cassert>
#include "bfgrid.hpp"
#include <functional>

using FrameBuffer = Map2D< std::wstring >;

struct GamePlan {
    GamePlan( Size s ) :
        buf( { ( s.w - 1 ) * 3 + 1, ( s.h - 1 ) * 2 + 1 }, { L' ' } ),
        size( s )
    {}

    void clearScreen() {
        std::wcout << "\033[2J\033[1;1H";
        return;
        for ( int i = 0; i != buf.height() + 2; i++ ) {
            std::wcout << L"\033[A\r";
        }
        std::wcout.flush();
    }

    void show() {
        std::wcout << "\n";
        for ( int y = 0; y != buf.height(); y++ ) {
            std::wcout << L"  ";
            for ( int x = 0; x != buf.width(); x++ )
                std::wcout << buf[ { x, y } ];
            std::wcout << "\n";
        }
        std::wcout << "\n";
        std::wcout.flush();
    }

    int vStep() {
        return buf.height() / ( size.h - 1 );
    }

    int hStep() {
        return buf.width() / ( size.w - 1 );
    }

    void drawGrid() {
        for ( int y = 0; y < buf.height(); y += vStep() ) {
            for ( int x = 0; x != buf.width(); x++ )
                buf[ { x, y } ] = L'-';
        }

        for ( int x = 0; x < buf.width(); x += hStep() ) {
            for ( int y = 0; y != buf.height(); y++ )
                buf[ { x, y } ] = L'|';
        }

        for ( int y = 0; y < buf.height(); y += vStep() ) {
            for ( int x = 0; x < buf.width(); x += hStep() ) {
                buf[ { x, y } ] = L'+';
            }
        }
    }

    Position gamePos( const Position& p ) {
        return { p.x * hStep(),  buf.height() - p.y * vStep() - 1 };
    }

    void drawRobot( RobotPosition pos ) {
        static std::map< Pred, wchar_t > icon{ {
            { Pred::North, L'^' },
            { Pred::South, L'v' },
            { Pred::East, L'>' },
            { Pred::West, L'<' }
        } };

        buf[ gamePos( pos ) ] =
            std::wstring( L"\033[1;31m" ) + icon[ pos.orient ] + L"\033[0m";
    }

    void drawKetchup( Position pos ) {
        buf[ gamePos( pos ) ] = L"\033[1;31m*\033[0m";
    }

    void drawEnemy( Position pos ) {
        if ( pos.x > 0 && pos.y > 0 )
            buf[ gamePos( pos ) ] = L"\033[1;31mX\033[0m";
    }

    void clear() {
        for ( int x = 0; x != buf.width(); x++ )
            for ( int y = 0; y != buf.height(); y++ )
                buf[ { x, y } ] = L" ";
    }

    FrameBuffer buf;
    Size size;
};

struct Robot {
    template < typename Yield >
    Robot( Yield y )
        : position( 0, 0, Pred::North ), yield( y )
    {}

    void dump() {
        std::wcout << "\r[ " << position.x << ", "
            << position.y << ", "
            << static_cast< int >( position.orient ) << " ]\n";
    }

    void clearDump() {
        std::wcout << "\033[A\r";
    }

    enum class State {
        KetchupDetected,
        PositionReached,
        RivalDetected,
        StopBtnPressed
    };

    State step(int numberOfSteps) {
        int *coord;
        int step = 1;
        switch( position.orient ) {
        case Pred::North:
            coord = &position.y;
            break;
        case Pred::West:
            coord = &position.x;
            step = -1;
            break;
        case Pred::South:
            coord = &position.y;
            step = -1;
            break;
        case Pred::East:
            coord = &position.x;
            break;
        default:
            assert( false );
        }

        for ( int i = 0; i != numberOfSteps; i++ ) {
            *coord += step;
            yield();
            auto res = ketchups.find( position );
            if ( res != ketchups.end() ) {
                ketchups.erase( res );
                return State::KetchupDetected;
            }
            if ( position == enemy )
                return State::RivalDetected;
        }

        return State::PositionReached;
    }

    void rotate( int degrees ) {
        int r = degrees / 90;
        int idx = static_cast< int >( position.orient ) + r + 4;
        idx %= 4;
        position.orient = static_cast< Pred >( idx );
        yield();
    }

    void openGate() {}
    void closeGate() {}

    RobotPosition position;
    std::function< void() > yield;

    std::set< Position > ketchups;
    Position enemy;
};