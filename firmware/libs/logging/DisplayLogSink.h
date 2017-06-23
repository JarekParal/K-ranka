#include "format.hpp"
#include <string>

using namespace std::string_literals;

template < class String >
class DisplayLogSinkT : public BaseLogSink< String > {
public:
    DisplayLogSinkT( ev3cxx::detail::Display &display, int width, int maxLines): display(display) 
    {
        _maxLines = maxLines;
        _width = width;

        if (maxLines < 2){
            maxLines = 2;
        }

        if (_width < 0){
            _width = 0;
        }

        display.resetScreen();

        header = format( "|{}|{}\n" )
            << string( "Level" ).width( LEVEL_WIDTH ).center()
            << string( "Message ").width( _width ).center();
        
        display.setFont(EV3_FONT_SMALL);
        for (auto ch : header){
            display.write(ch);
        }
        display.setFont(EV3_FONT_MEDIUM);
        // display.format(header);
    }

    virtual void log( Verbosity verb, const String& tag, const String& message,
        uint64_t timestamp ) override
    {
        std::string data;

        static std::vector< String > levels(
            { "panic", "error", "warni", "info", "debug" } );
        if ( verb >= PANIC && verb <= DEBUG ) {
            data = format( "|{}|{}\n" )
                << string( levels[ verb + 3] ).alignRight().width( LEVEL_WIDTH )
                << string( message ).alignLeft().width( _width ).clip();
        }
        else {
            data = format( "|{}|{}\n" )
                << number( static_cast< int >( verb ) ).width( LEVEL_WIDTH )
                << string( message ).alignLeft().width( _width ).clip();
        }

    display.setFont(EV3_FONT_SMALL);
        if(_line >= _maxLines){
            _line = 0;
            display.resetScreen();
            for (auto ch : header){
                display.write(ch);
            } 
        }
        _line++;

        // display.format(data);
        for (auto ch : data){
            display.write(ch);
        }

        display.setFont(EV3_FONT_MEDIUM);

    }
private:
    std::string header;
    ev3cxx::detail::Display &display;
    
    int _width;
    int _maxLines;
    int _line = 0;
    
    static constexpr const int TIME_WIDTH = 0;
    static constexpr const int LEVEL_WIDTH = 5;
    static constexpr const int TAG_WIDTH = 0;
};

using DisplayLogSink = DisplayLogSinkT<std::string>;
