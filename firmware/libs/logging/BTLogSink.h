#include "format.hpp"
#include <string>

using namespace std::string_literals;

template < class String >
class BTLogSinkT : public BaseLogSink< String > {
public:
    BTLogSinkT(ev3cxx::Bluetooth &bt, int width): bt(bt) 
    {
        _width = width;

        if (_width < 0){
            _width = 0;
        }


        std::string data = format( "| {} | {} | {} | {} |\n" )
            << string( "Time" ).width( TIME_WIDTH ).alignLeft()
            << string( "Level" ).width( LEVEL_WIDTH ).center()
            << string( "Tag" ).width( TAG_WIDTH ).center()
            << string( "Message ").width( _width ).center();
        
        for (auto ch : header){
            bt.write(ch);
        }
    }

    virtual void log( Verbosity verb, const String& tag, const String& message,
        uint64_t timestamp ) override
    {
        std::string data;

        static std::vector< String > levels( { "panic", "error", "warning", "info", "debug" } );
        if ( verb >= PANIC && verb <= DEBUG ) {
            data = format( "| {} | {} | {} | {} |\n" )
                << number( timestamp ).alignRight().width( TIME_WIDTH )
                << string( levels[ verb + 3] ).alignRight().width( LEVEL_WIDTH )
                << string( tag ).alignRight().width( TAG_WIDTH )
                << string( message ).alignLeft().width( _width ).clip();
        }
        else {
            data = format( "| {} | {} | {} | {} |\n" )
                << number( timestamp ).alignRight().width( TIME_WIDTH )
                << number( static_cast< int >( verb ) ).width( LEVEL_WIDTH )
                << string( tag ).alignRight().width( TAG_WIDTH )
                << string( message ).alignLeft().width( _width ).clip();
        }

        for (auto ch : data){
            bt.write(ch);
        }

    }
private:
    std::string header;
    ev3cxx::Bluetooth &bt;
    
    int _width;
    int _maxLines;
    int _line = 0;
    
    static constexpr const int TIME_WIDTH = 10;
    static constexpr const int LEVEL_WIDTH = 7;
    static constexpr const int TAG_WIDTH = 10;
};

using BTLogSink = BTLogSinkT<std::string>;
