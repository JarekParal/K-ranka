#pragma once

#include <vector>
#include <set>
#include <string>
#include <limits>

static const constexpr int inf = std::numeric_limits< int >::max();

enum class Pred { None, South, East, West, North };

struct Position {
    int x, y;

    bool operator==( const Position& o ) const {
        return x == o.x && y == o.y;
    }

    bool operator<( const Position& o ) const {
        if ( x < o.x )
            return true;
        return y < o.y;
    }
};

struct RobotPosition: public Position {
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
            std::vector< Destination >{ static_cast< size_t >( size.w ), t } }
    {}

    T& operator[]( const Position& p ) { return _map[ p.y ][ p.x ]; }
    const T& operator[]( const Position& p ) const { return _map[ p.y ][ p.x ]; }
    int width() const { return _map[ 0 ].size(); }
    int height() const { return _map.size(); }
    Size size() const { return { width(), height() }; }

    std::vector< std::vector< T > > _map;
};

using DestMap = Map2D< Destination >;

DestMap shortestPaths( RobotPosition from, Size size,
    std::set< Position > forbid = {} );

std::string visualizeDestMap( const DestMap& map );
