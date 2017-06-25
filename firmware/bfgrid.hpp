#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <limits>
#include <algorithm>

static const constexpr int inf = std::numeric_limits< int >::max();

enum class Pred { None = -1, North = 0, West, South, East };

std::ostream& operator<<( std::ostream& o, Pred p );

struct Position {
    int x, y;

    bool operator==( const Position& o ) const {
        return x == o.x && y == o.y;
    }

    bool operator<( const Position& o ) const {
        if ( x < o.x )
            return true;
        return x == o.x ? y < o.y : false;
    }
};

std::ostream& operator<<( std::ostream& o , Position p );

struct RobotPosition: public Position {
    RobotPosition() = default;
    RobotPosition( int x, int y, Pred o )
        : Position{ x, y }, orient( o )
    {}
    Pred orient;
};

struct Size {
    int w, h;
};

struct Destination {
    Pred pred;
    int distance;
};

template < typename T >
struct Map2D {
    Map2D( Size size, T t )
        : _map{ static_cast< size_t >( size.h ),
            std::vector< T >( static_cast< size_t >( size.w ), t ) }
    {}

    T& operator[]( const Position& p ) { return _map[ p.y ][ p.x ]; }
    const T& operator[]( const Position& p ) const { return _map[ p.y ][ p.x ]; }
    int width() const { return _map[ 0 ].size(); }
    int height() const { return _map.size(); }
    Size size() const { return { width(), height() }; }

    std::vector< std::vector< T > > _map;
};

struct DestMap;

DestMap shortestPaths( RobotPosition from, Size size,
    std::set< Position > forbid = {} );
std::vector< std::pair< Pred, int > > pathTo( Position pos, const DestMap& map );

std::string visualizeDestMap( const DestMap& map );

Pred invert( Pred p );

struct DestMap: public Map2D< Destination > {
    using Map2D::Map2D;


    std::vector< Pred > pathTo( Position pos ) {
        std::vector< Pred> path;
        while ( true ) {
            const auto& x = (*this)[ pos ];
            if ( x.distance == 0 )
                break;;
            path.push_back( invert( x.pred ) );
            pos = getPred( pos );
        }

    std::reverse( path.begin(), path.end() );
    return path;
}

    Position getPred( Position p ) {
        switch( ( *this )[ p ].pred ) {
            case Pred::North:
                return { p.x, p.y + 1 };
            case Pred::West:
                return { p.x - 1, p.y };
            case Pred::South:
                return { p.x, p.y - 1 };
            case Pred::East:
                return { p.x + 1, p.y };
            default:
                assert( false );
        }
        __builtin_unreachable();
    }
};
