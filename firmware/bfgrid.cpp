#include <cassert>
#include <utility>
#include <algorithm>

#include "bfgrid.hpp"

namespace {

int distance( DestMap& map, Position via, Position to ) {
    int dx = via.x - to.x;
    int dy = via.y - to.y;
    assert( abs( dx ) + abs( dy ) == 1 );

    auto& viaD = map[ via ];

    if ( viaD.distance == inf )
        return inf;

    Pred dir;
    if ( dx == -1 && dy == 0 )
        dir = Pred::West;
    else if ( dx == 1 && dy == 0 )
        dir = Pred::East;
    else if ( dx == 0 && dy == 1 )
        dir = Pred::North;
    else if ( dx == 0 && dy == -1 )
        dir = Pred::South;
    else
        assert( false );

    if ( dir == map[ to ].pred )
        return viaD.distance + 1;
    return viaD.distance + 2;
}

void relax( DestMap& map, Position p, const std::set< Position >& forbid ) {
    Size s = map.size();
    auto& destination = map[ p ];
    if ( forbid.find( p ) != forbid.end() )
        return;
    auto candidates = {
        std::make_pair( Position{ p.x + 1, p.y }, Pred::East ),
        std::make_pair( Position{ p.x - 1, p.y }, Pred::West ),
        std::make_pair( Position{ p.x, p.y + 1 }, Pred::North ),
        std::make_pair( Position{ p.x, p.y - 1 }, Pred::South )
    };
    for ( auto neighbour : candidates ) {
        auto n = neighbour.first;
        if ( forbid.find( n ) != forbid.end() )
            continue;
        if ( n.x < 0 || n.x >= s.w ||
             n.y < 0 || n.y >= s.h )
            continue;
        int d = distance( map, n, p );
        if ( destination.distance > d ) {
            destination.distance = d;
            destination.pred = neighbour.second;
        }
    }
}

} // namespace

DestMap shortestPaths( RobotPosition from, Size size, std::set< Position > forbid ) {
    DestMap map{ size, Destination{ Pred::None, inf } };
    map[ from ].distance = 0;

    for( int i = 0; i != 2 * ( size.w + size.h ); i++ )
        for ( int x = 0; x != size.w; x++ )
            for ( int y = 0; y != size.h; y++ )
                relax( map, { x, y }, forbid );
    return map;
}

std::string visualizeDestMap( const DestMap& map ) {
    std::string res;
    for( int y = 0; y != map.height(); y++ ) {
        std::string line;
        for ( int x = 0; x != map.width(); x++ ) {
            const Destination& d = map[ { x, y } ];
            assert( d.distance >= 0 );
            if ( d.distance == inf )
                line += ' ';
            else if ( d.distance == 0 )
                line += 'O';
            else switch( d.pred ) {
            case Pred::East:
                line += '>';
                break;
            case Pred::North:
                line += '^';
                break;
            case Pred::West:
                line += '<';
                break;
            case Pred::South:
                line += 'v';
                break;
            default:
                line += 'X';
            }
        }
        res = line + '\n' + res;
    }
    return res;
}